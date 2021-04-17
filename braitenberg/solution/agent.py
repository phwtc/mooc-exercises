#!/usr/bin/env python3

from dataclasses import dataclass
from typing import Optional, Tuple

import math
import numpy as np
from aido_schemas import (
    Context,
    DB20Commands,
    DB20Observations,
    EpisodeStart,
    GetCommands,
    JPGImage,
    LEDSCommands,
    protocol_agent_DB20,
    PWMCommands,
    RGB,
    wrap_direct,
)

import duckietown_code_utils as dcu
from connections import get_motor_left_matrix, get_motor_right_matrix
from preprocessing import preprocess


@dataclass
class BraitenbergAgentConfig:
    gain: float = 0.1
    const: float = 0.1


class BraitenbergAgent:
    config = BraitenbergAgentConfig()

    left: Optional[np.ndarray]
    right: Optional[np.ndarray]
    rgb: Optional[np.ndarray]
    l_max: float
    r_max: float
    l_min: float
    r_min: float

    def init(self, context: Context):
        context.info("init()")
        self.rgb = None
        self.l_max = -math.inf
        self.r_max = -math.inf
        self.l_min = math.inf
        self.r_min = math.inf
        self.left = None
        self.right = None
        self.count = 0
        self.state = 'straight'

    def on_received_seed(self, data: int):
        np.random.seed(data)

    def on_received_episode_start(self, context: Context, data: EpisodeStart):
        context.info(f'Starting episode "{data.episode_name}".')

    def on_received_observations(self, context: Context, data: DB20Observations):
        camera: JPGImage = data.camera
        if self.rgb is None:
            context.info("received first observations")
        self.rgb = dcu.bgr_from_rgb(dcu.bgr_from_jpg(camera.jpg_data))

    def compute_commands(self) -> Tuple[float, float]:
        """ Returns the commands (pwm_left, pwm_right) """
        # If we have not received any image, we don't move
        if self.rgb is None:
            return 0.0, 0.0

        if self.left is None:
            # if it is the first time, we initialize the structures
            shape = self.rgb.shape[0], self.rgb.shape[1]
            self.left = get_motor_left_matrix(shape)
            self.right = get_motor_right_matrix(shape)

        turn = 1
        # let's take only the intensity of RGB
        P = preprocess(self.rgb)
        # now we just compute the activation of our sensors
        l = float(np.sum(P * self.left))
        r = float(np.sum(P * self.right))

        # These are big numbers -- we want to normalize them.
        # We normalize them using the history

        # first, we remember the high/low of these raw signals
        self.l_max = max(l, self.l_max)
        self.r_max = max(r, self.r_max)
        self.l_min = min(l, self.l_min)
        self.r_min = min(r, self.r_min)

        # now rescale from 0 to 1
        ls = rescale(l, self.l_min, self.l_max)
        rs = rescale(r, self.r_min, self.r_max)

        self.count +=1


        gain = self.config.gain
        const = self.config.const
        gain = 0.2
        const = 0.1

        # Use a bit more information from self.left and self.right 
        # region (middle)
        width = 40
        dim = self.left.shape
        lp = P*self.left
        rp = P*self.right
        cl = float(np.sum(lp[:, dim[1]//2-width:dim[1]//2 +width]) / (dim[0] * width * 2) )
        thingleft  = float(np.sum(lp[:, dim[1]//8-width:dim[1]//8 +width]) / (dim[0] * width * 2) )
        thingright= float(np.sum(rp[:, dim[1]//8*7-width:dim[1]//8*7 +width]) / (dim[0] * width * 2) )

        if cl > 1.5 or ls > 0.9 or rs > 0.9 and not (ls > 0.9 and rs > 0.9): 
            self.state = 'turn'
            self.count = 0

            # either turn left / right
            if thingleft > thingright: 
                pwm_right = 0.9
                pwm_left  = 0.001
            else: 
                pwm_left= 0.9
                pwm_right= 0.001

        else: 

            if self.state == 'turn': 
                self.count += 1
                if self.count > 6: 
                    self.state = 'straight'
                    self.count = 0
                    gain = 0.2
                    const = 0.4
                gain = 0.1
                const = 0.01
            else: 
                gain = 0.2
                const = 0.4


            # # 5 straight, then back to normal 
            # if self.count > 5 and self.state == 'turn': 
            #     self.count = 0
            #     gain = 0.2
            #     const = 0.1
            #     # const = 0.2
            #     pwm_left = const + ls * gain
            #     pwm_right = const + rs * gain

            # else: 
            #     gain = 0.5
            #     const = 0.1
            #     # const = 0.1
            pwm_left = const + ls * gain
            pwm_right = const + rs * gain


        # # if we are really close, move away 
        # if abs(ls) > 0.9 or abs(rs) > 0.9:
        #     gain = 0.5
        #     const = 0.001
        # if self.count < 10: 
        #     gain = 0.01
        #     const = 0.05
        # pwm_left = const + ls * gain
        # pwm_right = const + rs * gain

        # if cl > 2: 
        #     # turn around
        #     # thing at left
        #     if thingright < thingleft: 
        #         pwm_right = 0.9
        #         pwm_left  = 0.001
        #     else: 
        #         pwm_left= 0.9
        #         pwm_right= 0.001

        print(f'cl: {cl:0.2f}, state: {self.state}')
        print(f'ls: {ls:0.2f}, l: {l:0.2f}, l_min: {self.l_min:0.2f}, l_max: {self.l_max:0.2f}, lc: {thingleft:0.2f}')
        print(f'rs: {rs:0.2f}, r: {r:0.2f}, r_min: {self.r_min:0.2f}, r_max: {self.r_max:0.2f}, rc: {thingright:0.2f}')
        print(f'gain: {gain:0.2f}, const: {const:0.2f}, left: {pwm_left:0.2f}, right: {pwm_right:0.2f}: ')

        self.l_max = max(l, self.l_max, r, self.r_max)
        self.r_max = self.l_max
        self.l_min = min(l, self.l_min, r, self.r_min)
        self.r_min = self.l_min
        return pwm_left, pwm_right

    def on_received_get_commands(self, context: Context, data: GetCommands):
        pwm_left, pwm_right = self.compute_commands()

        col = RGB(0.0, 0.0, 1.0)
        col_left = RGB(pwm_left, pwm_left, 0.0)
        col_right = RGB(pwm_right, pwm_right, 0.0)
        led_commands = LEDSCommands(col, col_left, col_right, col_left, col_right)
        pwm_commands = PWMCommands(motor_left=pwm_left, motor_right=pwm_right)
        commands = DB20Commands(pwm_commands, led_commands)
        context.write("commands", commands)

    def finish(self, context: Context):
        context.info("finish()")


def rescale(a: float, L: float, U: float):
    if np.allclose(L, U):
        return 0.0
    return (a - L) / (U - L)


def main():
    node = BraitenbergAgent()
    protocol = protocol_agent_DB20
    wrap_direct(node=node, protocol=protocol)


if __name__ == "__main__":
    main()
