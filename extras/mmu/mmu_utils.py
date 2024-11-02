# Happy Hare MMU Software
# Utility classes for Happy Hare MMU
#
# DebugStepperMovement
# Goal: Internal testing class for debugging synced movement
#
# PurgeVolCalculator
# Goal: Purge volume calculator based on color change
#
# Copyright (C) 2022  moggieuk#6538 (discord)
#                     moggieuk@hotmail.com
#
# (\_/)
# ( *,*)
# (")_(") Happy Hare Ready
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
import contextlib, math, time, threading, random, string


# Internal testing class for debugging synced movement
# Add this around move logic:
#     with DebugStepperMovement(self):
#        <synced_move>
class DebugStepperMovement:
    def __init__(self, mmu, debug=False):
        self.mmu = mmu
        self.debug = debug

    def __enter__(self):
        if self.debug:
            self.g_steps0 = self.mmu.gear_rail.steppers[0].get_mcu_position()
            self.g_pos0 = self.mmu.gear_rail.steppers[0].get_commanded_position()
            self.e_steps0 = self.mmu.mmu_extruder_stepper.stepper.get_mcu_position()
            self.e_pos0 = self.mmu.mmu_extruder_stepper.stepper.get_commanded_position()
            self.rail_pos0 = self.mmu.mmu_toolhead.get_position()[1]

    def __exit__(self, exc_type, exc_val, exc_tb):
        if self.debug:
            self.mmu.log_always("Waiting for movement to complete...")
            self.mmu.movequeues_wait()
            g_steps1 = self.mmu.gear_rail.steppers[0].get_mcu_position()
            g_pos1 = self.mmu.gear_rail.steppers[0].get_commanded_position()
            e_steps1 = self.mmu.mmu_extruder_stepper.stepper.get_mcu_position()
            e_pos1 = self.mmu.mmu_extruder_stepper.stepper.get_commanded_position()
            rail_pos1 = self.mmu.mmu_toolhead.get_position()[1]
            self.mmu.log_always("Gear steps: %d = %.4fmm, commanded movement: %.4fmm" % (g_steps1 - self.g_steps0, (g_steps1 - self.g_steps0) * self.mmu.gear_rail.steppers[0].get_step_dist(), g_pos1 - self.g_pos0))
            self.mmu.log_always("Extruder steps: %d = %.4fmm, commanded movement: %.4fmm" % (e_steps1 - self.e_steps0, (e_steps1 - self.e_steps0) * self.mmu.mmu_extruder_stepper.stepper.get_step_dist(), e_pos1 - self.e_pos0))
            self.mmu.log_always("Rail movement: %.4fmm" % (rail_pos1 - self.rail_pos0))


class PurgeVolCalculator:
    def __init__(self, min_purge_vol, max_purge_vol, multiplier):
        self.min_purge_vol = min_purge_vol
        self.max_purge_vol = max_purge_vol
        self.multiplier = multiplier

    def calc_purge_vol_by_rgb(self, src_r, src_g, src_b, dst_r, dst_g, dst_b):
        src_r_f = float(src_r) / 255.0
        src_g_f = float(src_g) / 255.0
        src_b_f = float(src_b) / 255.0
        dst_r_f = float(dst_r) / 255.0
        dst_g_f = float(dst_g) / 255.0
        dst_b_f = float(dst_b) / 255.0

        from_hsv_h, from_hsv_s, from_hsv_v = self.RGB2HSV(src_r_f, src_g_f, src_b_f)
        to_hsv_h, to_hsv_s, to_hsv_v = self.RGB2HSV(dst_r_f, dst_g_f, dst_b_f)
        hs_dist = self.DeltaHS_BBS(from_hsv_h, from_hsv_s, from_hsv_v, to_hsv_h, to_hsv_s, to_hsv_v)
        from_lumi = self.get_luminance(src_r_f, src_g_f, src_b_f)
        to_lumi = self.get_luminance(dst_r_f, dst_g_f, dst_b_f)

        lumi_purge = 0.0
        if to_lumi >= from_lumi:
            lumi_purge = math.pow(to_lumi - from_lumi, 0.7) * 560.0
        else:
            lumi_purge = (from_lumi - to_lumi) * 80.0

        inter_hsv_v = 0.67 * to_hsv_v + 0.33 * from_hsv_v
        hs_dist = min(inter_hsv_v, hs_dist)
        hs_purge = 230.0 * hs_dist

        purge_volume = self.calc_triangle_3rd_edge(hs_purge, lumi_purge, 120.0)
        purge_volume = max(purge_volume, 0.0)
        purge_volume += self.min_purge_vol
        purge_volume *= self.multiplier
        purge_volume = min(int(purge_volume), self.max_purge_vol)

        return purge_volume

    def calc_purge_vol_by_hex(self, src_clr, dst_clr):
        return self.calc_purge_vol_by_rgb(*self.hex_to_rgb(src_clr), *self.hex_to_rgb(dst_clr))

    @staticmethod
    def RGB2HSV(r, g, b):
        Cmax = max(r, g, b)
        Cmin = min(r, g, b)
        delta = Cmax - Cmin

        if abs(delta) < 0.001:
            h = 0.0
        elif Cmax == r:
            h = 60.0 * math.fmod((g - b) / delta, 6.0)
        elif Cmax == g:
            h = 60.0 * ((b - r) / delta + 2)
        else:
            h = 60.0 * ((r - g) / delta + 4)
        s = 0.0 if abs(Cmax) < 0.001 else delta / Cmax
        v = Cmax
        return h, s, v

    @staticmethod
    def to_radians(degree):
        return degree / 180.0 * math.pi

    @staticmethod
    def get_luminance(r, g, b):
        return r * 0.3 + g * 0.59 + b * 0.11

    @staticmethod
    def calc_triangle_3rd_edge(edge_a, edge_b, degree_ab):
        return math.sqrt(edge_a * edge_a + edge_b * edge_b - 2 * edge_a * edge_b * math.cos(PurgeVolCalculator.to_radians(degree_ab)))

    @staticmethod
    def DeltaHS_BBS(h1, s1, v1, h2, s2, v2):
        h1_rad = PurgeVolCalculator.to_radians(h1)
        h2_rad = PurgeVolCalculator.to_radians(h2)

        dx = math.cos(h1_rad) * s1 * v1 - math.cos(h2_rad) * s2 * v2
        dy = math.sin(h1_rad) * s1 * v1 - math.sin(h2_rad) * s2 * v2
        dxy = math.sqrt(dx * dx + dy * dy)

        return min(1.2, dxy)

    @staticmethod
    def hex_to_rgb(hex_color):
        hex_color = hex_color.lstrip('#')
        if len(hex_color) == 3:
            hex_color = ''.join([c * 2 for c in hex_color])
        if len(hex_color) != 6:
            raise ValueError("Invalid hex color code, it should be 3 or 6 digits long")
        color_value = int(hex_color, 16)
        r = (color_value >> 16) & 0xFF
        g = (color_value >> 8) & 0xFF
        b = color_value & 0xFF
        return r, g, b

# TODO: this may be large/complex enough to warrant moving to
#       it's own file
class InteractivePrompt:
    TYPE_TEXT = 'text'
    TYPE_BUTTON = 'button'
    TYPE_FOOTER_BUTTON = 'footer_button'

    COLOR_DEFAULT = None
    COLOR_PRIMARY = 'primary'
    COLOR_SECONDARY = 'secondary'
    COLOR_INFO = 'info'
    COLOR_WARNING = 'warning'
    COLOR_ERROR = 'error'
    COLORS = [COLOR_DEFAULT, COLOR_PRIMARY, COLOR_SECONDARY, COLOR_INFO, COLOR_WARNING, COLOR_ERROR]

    def __init__(self, mmu):
        self.mmu = mmu
        self.gcode = mmu.gcode
        self.reactor = mmu.reactor
        self.mmu_machine = mmu.mmu_machine
        self._is_open = False
        self.reset()

        # Counter to track the number of messages since the prompt began
        # this is to work around a bug in mainsail where it only displays
        # the dialog if the prompt messages are within the last 100 messages.
        # When we have seen nearly 100 messages we will want to re-show
        # the prompt to ensure it stays visible.
        # TODO: implement re-show
        self._msg_count_since_prompt = 0

        self.gcode.register_command('__MMU_INTERACTIVE_RESPOND', self.cmd_MMU_INTERACTIVE_RESPOND, desc = self.cmd_MMU_INTERACTIVE_RESPOND_help)
        
        self.gcode.register_output_handler(self.gcode_msg_handler)

    def gcode_msg_handler(self, msg):
        if msg.startswith('// action:prompt_begin'):
            self._msg_count_since_prompt = 0
        else:
            self._msg_count_since_prompt += 1
            # Workaround for bug in Mainsail where dialog automatically
            # closes if the prompt config is not in the last 100 messages
            # if self._msg_count_since_prompt > 70 and self._is_open:
            #     self.reactor.register_callback(lambda et: self.show())

        if msg.startswith('// action:prompt_show'):
            self._is_open = True

        elif msg.startswith('// action:prompt_end'):
            self.mmu.log_always("prompt was closed %s" % msg)
            self._is_open = False

    def run_on_main(self, cb, wait=True):
        completion = self.reactor.register_callback(lambda et: cb())
        if wait:
            while not completion.test():
                time.sleep(0.3)

    def run_script_from_command(self, cmd, wait=True):
        self.run_on_main(lambda et: self.gcode.run_script_from_command(cmd), wait=wait)
        

    @contextlib.contextmanager
    def interactive_prompt(self, new_action):
        old_action = self._set_action(new_action)
        try:
            yield (old_action, new_action)
        finally:
            self._set_action(old_action)

    def next_response(self):
        self._response = None
        num = 0
        while self._is_open:
            # self.mmu.log_always("Waiting for response (id=%d) %d. is_open=%s. has response %s" % (0, num, self._is_open, self._response is not None))
            if self._response is not None:
                yield self._response
                self._response = None
            num += 1
            time.sleep(.3)
        yield (None, None)

    def response(self):
        for result in self.next_response():
            return result

    cmd_MMU_INTERACTIVE_RESPOND_help = "Interactive respond"
    def cmd_MMU_INTERACTIVE_RESPOND(self, gcmd):
        self.mmu.log_debug("Interactive response %s" % gcmd.get_commandline())
        id = gcmd.get('CB')
        if id in self._callbacks:
            self._response = self._callbacks[id]()

    def set_title(self, title):
        self._prompt_title = title
        return self

    def add_text(self, text):
        self._prompt_items.append((self.TYPE_TEXT, [text]))
        return self

    def add_button(self, label, value=None, callback=None, color=None, id=None):
        return self._add_button(
            self.TYPE_BUTTON,
            label,
            value=value,
            callback=callback,
            color=color,
            id=id
        )

    def add_footer_button(self, label, value=None, callback=None, color=None, id=None):
        return self._add_button(
            self.TYPE_FOOTER_BUTTON,
            label,
            value=value,
            callback=callback,
            color=color,
            id=id
        )
    
    def add_button_group(self, *btns):
        self._prompt_items.append(('button_group_start', None))
        for kwargs in btns:
            self.add_button(**kwargs)
        self._prompt_items.append(('button_group_end', None))
        return self

    def create_button(self, label, value=None, callback=None, color=None, id=None):
        return {
            'label': label,
            'value': value,
            'callback': callback,
            'color': color,
            'id': id
        }

    def _add_button(self, type, label, value=None, callback=None, color=None, id=None):
        cb_id = str(id if id is not None else len(self._callbacks))
        value = label if value is None else value
        self._callbacks[cb_id] = lambda: self._button_callback(cb_id, callback if callback is not None else (lambda id: value))
        gcode = '__MMU_INTERACTIVE_RESPOND CB=\'%s\'' % cb_id
        params = [label, gcode]
        if color is not None:
            params.append(color)
        self._prompt_items.append((type, params))
        return self
    
    def _button_callback(self, id, callback):
        response = None if callback is None else callback(id)
        return response

    @contextlib.contextmanager
    def wrap(self):
        try:
            self.reset()
            yield self
        finally:
            self.close()

    @contextlib.contextmanager
    def wrap_show(self):
        try:
            self.show()
            yield self
        finally:
            self.close()

    def show(self):
        cmd = self._build_prompt()
        self.mmu.gcode.run_script_from_command(cmd)
        return self

    def close(self):
        if self._is_open:
            self.mmu.gcode.run_script_from_command('RESPOND TYPE=command MSG="action:prompt_end"')
        return self

    def reset(self):
        self._prompt_title = None
        self._prompt_items = []
        self._callbacks = {}
        self._response = None
        return self

    def _build_prompt(self):
        cmds = [
            'action:prompt_end',
            'action:prompt_begin %s' % self._prompt_title,
        ]
        for type, params in self._prompt_items:
            cmd = 'action:prompt_%s %s' % (type, '' if params is None else '|'.join(params))
            cmds.append(cmd.strip())
        cmds.append('action:prompt_show')
        return '\n'.join(map(lambda cmd: 'RESPOND TYPE=command MSG="%s"' % cmd, cmds))
