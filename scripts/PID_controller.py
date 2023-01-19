
class controllers:

    def __init__(self):

        self.x = 0
        self.y = 0
        self.xd = 320/2
        self.yd = 240/2
        self.dt = 1/30

        self.kpx = 10
        self.kdx = 0
        self.kix = 0
        self.e_x_I = 0
        self.e_x_dot = 0
        self.e_x = 0
        self.Ex = 0
        
        self.kpy = 10
        self.kdy = 0
        self.kiy = 0
        self.e_y_I = 0
        self.e_y_dot = 0
        self.e_y = 0
        self.Ey = 0
    def update_centroid(self,x,y):
        self.x = x
        self.y = y

    def obtain_commands_yaw(self):
        self.e_x = self.xd - self.x
        self.e_x_dot = (self.e_x-self.e_x0)/self.dt
        self.e_x0 = self.e_x
        self.e_x_I = self.e_x_I + self.e_x*self.dt
        self.Ex = self.kpx * self.e_x + self.kdx* self.e_x_dot + self.kix * self.e_x_I
        return self.Ex
    def obtain_commands_heave(self):
        self.e_y = self.yd - self.y
        self.e_y_dot = (self.e_y-self.e_y0)/self.dt
        self.e_y0 = self.e_y
        self.e_y_I = self.e_y_I + self.e_y*self.dt

        self.Ey = self.kpy * self.e_y + self.kdy * self.e_y_dot + self.kiy * self.e_y_I
        return self.Ey


        
