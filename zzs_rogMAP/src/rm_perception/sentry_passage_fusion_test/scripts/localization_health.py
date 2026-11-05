"""Point-LIO diagnostic contract and UUID-specific navigation cancellation.

No clearance/terrain policy and no motor publishing live in this module.
"""
import math
import time


class LocalizationHealth:
    def __init__(self):
        self.session = None
        self.sequence = 0
        self.generation = 0
        self.changed = False
        self.tracking = False
        self.received = -math.inf
        self.last_valid = 0.

    def update(self, message, now=None):
        now = time.monotonic() if now is None else now
        try:
            if len(message.status) != 1:
                raise ValueError('Expected one localization status')
            status = message.status[0]
            fields = {v.key: v.value for v in status.values}
            if (status.name != 'pointlio/localization' or not status.hardware_id or
                    len(fields) != len(status.values)):
                raise ValueError('Invalid localization identity')
            seq = int(fields['sequence'])
            stamp = float(fields['last_valid_stamp'])
            generation = int(fields['generation'])
            state = fields['state']
            if seq <= 0 or generation < 0 or not math.isfinite(stamp) or stamp < 0 or state not in (
                    'INITIALIZING', 'TRACKING', 'DEGRADED', 'NEEDS_RESET'):
                raise ValueError('Invalid localization fields')
            if self.session is None:
                self.session = status.hardware_id
            if self.session != status.hardware_id:
                self.changed = True
            if self.changed:
                self.tracking = False
                return
            if seq <= self.sequence:
                return  # replay does not renew the watchdog
            if generation < self.generation:
                raise ValueError("generation went backwards")
            self.generation = generation
            self.sequence = seq
            self.received = now
            self.last_valid = stamp
            published = message.header.stamp.sec+message.header.stamp.nanosec*1e-9
            self.tracking = (state == 'TRACKING' and status.level in (0, b'\x00') and
                             stamp > 0 and -.1 <= published-stamp <= .25)
        except (ValueError, KeyError, TypeError, OverflowError):
            self.tracking = False

    def ready(self, now=None):
        now = time.monotonic() if now is None else now
        return self.tracking and not self.changed and 0 <= now-self.received <= .25


class NavigationRecovery:
    """Cancel only old UUIDs; never let a late cancel response cancel a new goal."""
    def __init__(self, node):
        from action_msgs.srv import CancelGoal
        self.request_type = CancelGoal.Request
        self.client = node.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        self.node = node
        self.health = LocalizationHealth()
        self.was_ready = False
        self.generation = 0
        self.cutoff_ns = node.get_clock().now().nanoseconds
        self.seen_status = False
        self.active = {}
        self.retired = set()
        self.futures = {}
        self.last_attempt = {}
        self.reason = 'waiting for localization'

    def on_status(self, message):
        self.seen_status = True
        self.active = {}
        for status in message.status_list:
            key = bytes(status.goal_info.goal_id.uuid)
            if status.status in (4, 5, 6):
                self.retired.discard(key)
                self.futures.pop(key, None)
                continue
            if status.status in (1, 2, 3):
                stamp = status.goal_info.stamp
                ns = stamp.sec*1000000000+stamp.nanosec
                self.active[key] = ns
                if not self.health.ready() or ns <= self.cutoff_ns:
                    self.retired.add(key)

    def tick(self):
        healthy = self.health.ready()
        if healthy != self.was_ready or self.health.generation != self.generation:
            self.generation = self.health.generation
            self.cutoff_ns = self.node.get_clock().now().nanoseconds
            self.retired.update(self.active)
            self.was_ready = healthy
        if not healthy:
            self.retired.update(self.active)
        now = time.monotonic()
        for key in list(self.retired):
            pending = self.futures.get(key)
            if pending is not None:
                future, started = pending
                if future.done():
                    try:
                        response = future.result()
                        self.reason = ('waiting for goal terminal status' if response.return_code == 0
                                       else 'cancel rejected; motion remains disabled')
                    except Exception as error:
                        self.reason = 'cancel failed: '+str(error)
                    self.futures.pop(key, None)
                elif now-started < 2.:
                    continue
                else:
                    future.cancel()
                    self.futures.pop(key, None)
                    self.reason = 'cancel timed out; motion remains disabled'
            if now-self.last_attempt.get(key, -math.inf) < 1.:
                continue
            if not self.client.service_is_ready():
                self.reason = 'cancel service unavailable; motion remains disabled'
                continue
            request = self.request_type()
            request.goal_info.goal_id.uuid = list(key)
            self.last_attempt[key] = now
            try:
                self.futures[key] = (self.client.call_async(request), now)
            except Exception as error:
                self.reason = 'cancel transport failed; motion remains disabled: '+str(error)
        allowed = healthy and self.seen_status and not self.retired and any(
            stamp > self.cutoff_ns for stamp in self.active.values())
        if not healthy:
            self.reason = 'localization unavailable or session changed'
        elif allowed:
            self.reason = 'new goal in healthy localization session'
        return allowed
