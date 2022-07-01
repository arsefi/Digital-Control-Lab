class PID():
    #first, we have to initialize the controller
	def __init__(self,KP,KI,KD,target,max,min):
		self.kp = KP
		self.ki = KI
		self.kd = KD		
		self.sp = target
		self.error_last = 0
		self.integral_error = 0
		self.saturation_max = max #max speed of the robot = +10
		self.saturation_min = min #min speed of the robot = -10
        
    #using the controller
	def compute(self,pos,dt):
		error = self.sp - pos #compute the error
		derivative_error = (error - self.error_last) / dt #find the derivative of the error (how the error changes with time)
		self.integral_error += error * dt #error build up over time
		output = self.kp*error + self.ki*self.integral_error + self.kd*derivative_error 
		self.error_last = error
		if output > self.saturation_max:
			output = self.saturation_max
		elif output < self.saturation_min:
			output = self.saturation_min
		return output