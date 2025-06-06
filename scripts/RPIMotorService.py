#!/usr/bin/env python3

import RPi.GPIO as GPIO
import time
import threading
import numpy as np
import math

from concurrent import futures
import logging

import grpc

import rpi_motor_pb2
import rpi_motor_pb2_grpc

np.set_printoptions(formatter={'float': lambda x: "{0:0.2f}".format(x)})

class RPIMotorServiceImpl(rpi_motor_pb2_grpc.RPIMotorServicer):

    def __init__(self):
        GPIO.setmode(GPIO.BOARD)

        self.count = 0
        self.new_cmd = False
        self.chan_list = [33, 35, 36, 37, 38, 40, 23, 24, 21, 7]
        self.enc_list =  [13, 15, 16, 18, 29, 31]

        self.enc = np.array([0, 0, 0], dtype=np.float64)
        self.prev_enc = [0, 0, 0]
        self.vel = np.array([0, 0, 0], dtype=np.float64)
        self.prev_vel = np.array([0, 0, 0], dtype=np.float64)
        self.enc_last = ["00", "00", "00"]
        self.states = {"0001":1, "0010":-1, "0100":-1, "0111":1, "1000":1, "1011":-1, "1101":-1, "1110":1}
        self.err_hist = []

        self.p_gain = np.array([42.5, 42.5, 42.5], dtype=np.float64)
        self.i_gain = np.array([0.0001, 0.0001, 0.0001], dtype=np.float64)
        self.d_gain = np.array([0.000001, 0.000001, 0.000001], dtype=np.float64)
        self.cut_vel = np.array([0.055, 0.055, 0.055], dtype=np.float64)

        self.ppr = 4*80*1 # 4 pulses per motor rev., 80 motor rev. = 1 wheel rev.
        self.duty = np.array([0, 0, 0], dtype=np.float64)
        # Wheel angular velocities (target/commanded values)
        self.w = np.array([0, 0, 0], dtype=np.float64)

        # Robot physical parameters
        self.r = 0.0240  # Wheel radius in meters
        self.R = 0.1041  # Distance from center to wheel in meters
        
        # Define wheel angles in degrees
        self.alpha_1 = 30
        self.alpha_2 = 150
        self.alpha_3 = 270

        # Build forward kinematics matrix M
        try:
            alphas_rad = np.deg2rad([self.alpha_1, self.alpha_2, self.alpha_3])
            M = np.array([
                [np.cos(alphas_rad[0] + np.pi/2), np.cos(alphas_rad[1] + np.pi/2), np.cos(alphas_rad[2] + np.pi/2)],
                [np.sin(alphas_rad[0] + np.pi/2), np.sin(alphas_rad[1] + np.pi/2), np.sin(alphas_rad[2] + np.pi/2)],
                [1.0, 1.0, 1.0]
            ], dtype=np.float64)
            
            # Calculate inverse kinematics matrix (Minv)
            self.Minv = np.linalg.inv(M)
            print("Inverse kinematics matrix calculated successfully:")
            print(self.Minv)
        except np.linalg.LinAlgError as e:
            print(f"ERROR calculating inverse kinematics matrix: {e}.")
            print("Using identity matrix instead. Robot control will not function correctly!")
            self.Minv = np.identity(3, dtype=np.float64)

        self.pwm = []
        self.freq = 200.0

        self.can_display = True
        self.can_control = True
        self.stop_count = 0

        self.v_x = 0.00
        self.v_y = 0.00
        self.v_yaw = 0.00
        self.v_heave = 0.00
        self.pitch = 0.00
        self.leds = 0.00

        GPIO.setup(self.chan_list, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.enc_list, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        GPIO.add_event_detect(13, GPIO.BOTH, callback=self.encoder_0_cbk)
        GPIO.add_event_detect(15, GPIO.BOTH, callback=self.encoder_0_cbk)
        GPIO.add_event_detect(16, GPIO.BOTH, callback=self.encoder_1_cbk)
        GPIO.add_event_detect(18, GPIO.BOTH, callback=self.encoder_1_cbk)
        GPIO.add_event_detect(29, GPIO.BOTH, callback=self.encoder_2_cbk)
        GPIO.add_event_detect(31, GPIO.BOTH, callback=self.encoder_2_cbk)

        self.pwm.append(GPIO.PWM(38, self.freq))
        self.pwm.append(GPIO.PWM(40, self.freq))
        self.pwm.append(GPIO.PWM(36, self.freq))
        self.pwm.append(GPIO.PWM(37, self.freq))
        self.pwm.append(GPIO.PWM(33, self.freq))
        self.pwm.append(GPIO.PWM(35, self.freq))
        self.pwm.append(GPIO.PWM(23, self.freq)) # elevator dc pwm1
        self.pwm.append(GPIO.PWM(24, self.freq)) # elevator dc pwm2
        self.pwm.append(GPIO.PWM(21, 50)) # servo pwm
        self.pwm.append(GPIO.PWM(7, 200)) # LEDs pwm

        for idx in range(0, 3):
            self.err_hist.append([])
            self.pwm[idx*2].start(0.0)
            self.pwm[idx*2+1].start(0.0)

        self.pwm[6].start(0.0)
        self.pwm[7].start(0.0)
        self.pwm[8].start(0.0)
        self.pwm[9].start(0.0)

        self.dp = threading.Thread(target=self.display_stats)
        self.dp.start()
        self.ct = threading.Thread(target=self.control_loop)
        self.ct.start()

        print("GPIO initialized")

    def map_range(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min

    def encoder_0_cbk(self, chan):
        curr = str(GPIO.input(13)) + str(GPIO.input(15))
        key = self.enc_last[0] + curr
        if key in self.states:
            drctn = self.states[key]
            self.enc_last[0] = curr
            self.enc[0] += float(drctn)
        if self.enc[0] > 100000000:
            self.enc[0] -= 100000000
        if self.enc[0] < -100000000:
            self.enc[0] += 100000000

    def encoder_1_cbk(self, chan):
        curr = str(GPIO.input(16)) + str(GPIO.input(18))
        key = self.enc_last[1] + curr
        if key in self.states:
            drctn = self.states[key]
            self.enc_last[1] = curr
            self.enc[1] += float(drctn)
        if self.enc[1] > 100000000:
            self.enc[1] -= 100000000
        if self.enc[1] < -100000000:
            self.enc[1] += 100000000

    def encoder_2_cbk(self, chan):
        curr = str(GPIO.input(29)) + str(GPIO.input(31))
        key = self.enc_last[2] + curr
        if key in self.states:
            drctn = self.states[key]
            self.enc_last[2] = curr
            self.enc[2] += float(drctn)
        if self.enc[2] > 100000000:
            self.enc[2] -= 100000000
        if self.enc[2] < -100000000:
            self.enc[2] += 100000000

    def display_stats(self):
        while(self.can_display):
            print("cmd: " + str(self.w) + " / pls: " + str(self.enc) + " / spd: " + str(self.vel) + " / dty: " + str(self.duty) + " / stp: " + str(self.stop_count))
            time.sleep(0.25)

    def control_loop(self):
        self.stop_count = 0
        while(self.can_control):
            if self.new_cmd:
                self.new_cmd = False
                self.stop_count = 0
            else:
                self.stop_count += 1

            if self.stop_count > 50:
               self.stop_count = 50
               self.v_x = 0.0
               self.v_y = 0.0
               self.v_yaw = 0.0
               self.v_heave = 0.0
               self.pitch = 0.0
               self.leds = 0.0
               self.w[0] = 0.0
               self.w[1] = 0.0
               self.w[2] = 0.0

            for idx in range(0, 3):
                self.prev_vel[idx] = self.vel[idx]
                self.vel[idx] = (self.prev_vel[idx] + 2*np.pi*(self.enc[idx] - self.prev_enc[idx])/self.ppr)/2.0
                if self.vel[idx] < self.cut_vel[idx] and self.vel[idx] > -self.cut_vel[idx]:
                    self.vel[idx] = 0.0
                self.prev_enc[idx] = self.enc[idx]
                if len(self.err_hist[idx]) > 20:
                    self.err_hist[idx].pop(0)
                self.err_hist[idx].append(self.vel[idx] - self.w[idx])
                p_err = self.err_hist[idx][len(self.err_hist[idx])-1]
                i_err = np.sum(self.err_hist[idx])
                d_err = (self.err_hist[idx][len(self.err_hist[idx])-1] - self.err_hist[idx][len(self.err_hist[idx])-2])/2.0

                output = p_err*self.p_gain[idx] + i_err*self.i_gain[idx] + d_err*self.d_gain[idx]
                self.duty[idx] = output

                if self.duty[idx] > 100.0:
                        self.duty[idx] = 100.0
                elif self.duty[idx] < -100.0:
                        self.duty[idx] = -100.0

                if self.duty[idx] == 0.0:
                    self.pwm[idx*2].ChangeDutyCycle(0.0)
                    self.pwm[idx*2+1].ChangeDutyCycle(0.0)
                elif self.duty[idx] < 0.0:
                    self.pwm[idx*2].ChangeDutyCycle(0.0)
                    self.pwm[idx*2+1].ChangeDutyCycle(-self.duty[idx])
                else:
                    self.pwm[idx*2].ChangeDutyCycle(self.duty[idx])
                    self.pwm[idx*2+1].ChangeDutyCycle(0.0)
            

            if self.v_heave == 0.0:
                self.pwm[6].ChangeDutyCycle(0.0)
                self.pwm[6+1].ChangeDutyCycle(0.0)
            elif self.v_heave > 0.0:
                self.pwm[6].ChangeDutyCycle(0.0)
                self.pwm[6+1].ChangeDutyCycle(100.0)
            else:
                self.pwm[6].ChangeDutyCycle(100.0)
                self.pwm[6+1].ChangeDutyCycle(0.0)
            
            if self.count % 1 == 0:
                self.pwm[8].ChangeDutyCycle(self.pitch)
                self.pwm[9].ChangeDutyCycle(100.0)
            self.count += 1
            time.sleep(0.01)

    def SetState(self, request, context):
        result = rpi_motor_pb2.StateReply()

        try:
            # Store requested velocity values
            self.v_x = request.vel_x
            self.v_y = request.vel_y
            self.v_yaw = request.vel_yaw
            self.v_heave = request.vel_heave
            self.pitch = request.pitch
            self.leds = request.leds

            # Create the velocity vector v = [vx, vy, ω]
            v = np.array([self.v_x, self.v_y, self.v_yaw], dtype=np.float64)
                        
            # Calculate linear wheel speeds using inverse kinematics (s = Minv * v)
            s = np.dot(self.Minv, v)

            # Convert linear wheel speeds to angular velocities (w = s/r)
            for i in range(3):
                self.w[i] = s[i] / self.r
            
            self.new_cmd = True
            result.res = True
            
        except Exception as e:
            print("ERROR: " + str(e))
            result.res = False

        return result

    def __del__(self):
        self.can_display = False
        self.can_control = False
        self.dp.join()
        self.ct.join()
        for mtr in self.pwm:
            mtr.stop()

        GPIO.remove_event_detect(13)
        GPIO.remove_event_detect(15)
        GPIO.remove_event_detect(16)
        GPIO.remove_event_detect(18)
        GPIO.remove_event_detect(29)
        GPIO.remove_event_detect(31)

        time.sleep(1)

        GPIO.cleanup()

        print("GPIO destroyed")

def serve():
    port = 50201
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=3))
    rpi_motor_pb2_grpc.add_RPIMotorServicer_to_server(RPIMotorServiceImpl(), server)
    server.add_insecure_port('[::]:'+str(port))
    server.start()
    print("RPIMotor service started at " + str(port))
    server.wait_for_termination()

if __name__ == '__main__':
    logging.basicConfig()
    serve()