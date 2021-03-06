import math, logging
from pool import PseudoMotorController


def rotate_x(y, z, cosangle, sinangle):
    """3D rotaion around *x* (pitch). *y* and *z* are values or arrays.
    Positive rotation is for positive *sinangle*. Returns *yNew, zNew*.
    
    :param y: (float or list<float>) value of the y coordinate 
    :param z: (float or list<float>) value of the z coordinate 
    :param cosangle: (float) cosinus of the rotation angle
    :param sinangle: (float) sinus of the rotation angle

    :return: (tuple<float>) new (rotated) y and z"""
    return cosangle * y - sinangle * z, sinangle * y + cosangle * z

def rotate_y(x, z, cosangle, sinangle):
    """3D rotaion around *y* (roll). *x* and *z* are values or arrays.
    Positive rotation is for positive *sinangle*. Returns *xNew, zNew*.
    
    :param x: (float or list<float>) value of the x coordinate 
    :param z: (float or list<float>) value of the z coordinate 
    :param cosangle: (float) cosinus of the rotation angle
    :param sinangle: (float) sinus of the rotation angle

    :return: (tuple<float>) new (rotated) x and z"""
    return cosangle * x + sinangle * z, -sinangle * x + cosangle * z

def rotate_z(x, y, cosangle, sinangle):
    """3D rotaion around *z*. *x* and *y* are values or arrays.
    Positive rotation is for positive *sinangle*. Returns *xNew, yNew*.

    :param x: (float or list<float>) value of the x coordinate 
    :param y: (float or list<float>) value of the y coordinate 
    :param cosangle: (float) cosinus of the rotation angle
    :param sinangle: (float) sinus of the rotation angle

    :return: (tuple<float>) new (rotated) x and y"""
    return cosangle * x - sinangle * y, sinangle * x + cosangle * y


class TwoXStageController(PseudoMotorController):
    """This is a pseudomotor controller for a stage with two lateral translation motors.
    It expects two physical motors: mx1, mx2 and provides 2 pseudomotors: x, yaw.
    Motor mx1 is the upstream one, and yaw angle is increasing with increasing its position.
     
    It requires definition of 3 properties:
    Tx1Coordinates - a string representing Tx1 x,y coordinates in local system e.g. "-711.9, 0" 
    Tx2Coordinates - a string representing Tx2 x,y coordinates in local system e.g. "689, 0"
    Dx - nominal x shift of the center in local system."""

    pseudo_motor_roles = ('x', 'yaw')
    motor_roles = ('mx1', 'mx2')

    class_prop = {'Tx1Coordinates' : {'Type' : 'PyTango.DevString', 'Description' : 'tx1 coordination: x,y in local system'},
                  'Tx2Coordinates' : {'Type' : 'PyTango.DevString', 'Description' : 'tx2 coordination: x,y in local system'},
                  'Dx'             : {'Type' : 'PyTango.DevDouble', 'Description' : 'nominal x shift of the center in local system', 'DefaultValue' : 0}}

    def __init__(self, inst, props):  
        PseudoMotorController.__init__(self, inst, props)
        #self._log.setLevel(logging.DEBUG)

        try:
            self.tx1 = [float(c) for c in props['Tx1Coordinates'].split(',')]
            self.tx2 = [float(c) for c in props['Tx2Coordinates'].split(',')]
            self.dx = float(props['Dx'])
            
            if len(self.tx1) != 2 or len(self.tx2) != 2:
                raise ValueError("Tx1 and Tx2 Coordinates properties must be x,y coordinates in local system")
            
            if self.tx1[1] == self.tx2[1]:
                raise ValueError('The mirror must be initially horizontal!')
            
        except ValueError, e:
            self._log.error('Could not parse class properties to generate coordinates.')
            raise e
        
    def calc_physical(self, index, pseudos):
        return self.calc_all_physical(pseudos)[index - 1]

    def calc_pseudo(self, index, physicals):
        return self.calc_all_pseudo(physicals)[index - 1]

    def calc_all_physical(self, pseudos):
        x, yaw = pseudos 
        tanYaw = math.tan(yaw/1000)#converts back to radians
        tx1 = -tanYaw * self.tx1[1] + x
        tx2 = -tanYaw * self.tx2[1] + x
        return tx1, tx2

    def calc_all_pseudo(self, physicals):
        tx1, tx2 = physicals
        x = tx1 - (tx2 - tx1) * self.tx1[1] / (self.tx2[1] - self.tx1[1])
        yaw = -math.atan((tx2 - tx1) / (self.tx2[1] - self.tx1[1]))
        yaw *= 1000 # conversion to mrad
        return x, yaw


class TripodTableController(PseudoMotorController):
    """This is a pseudomotor controller for a three-legs table.
    It expects three physical motors: jack1, jack2, jack3 and provides 3 pseudomotors: z, pitch and roll.
    Jack1 is the most upstream one and Jack3 is the most downstream. If two of the jacks 
    have the same distance to the source the left one comes first. 
     
    It requires definition of 4 properties (all of them are strings with comma separated float values
    representing x,y,z coordinates in global coordinate system e.g. "3123.09, -3232.33, 1400"): 
    Jack1Coordinates, Jack2Coordinates, Jack3Coordinates, CenterCoordinates"""
    
    pseudo_motor_roles = ('z', 'pitch', 'roll')
    motor_roles = ('jack1', 'jack2', 'jack3')

    class_prop = { 'Jack1Coordinates' : {'Type' : 'PyTango.DevString', 'Description' : 'jack1 coordination: x,y,z'},
                         'Jack2Coordinates' : {'Type' : 'PyTango.DevString', 'Description' : 'jack2 coordination: x,y,z'},
                         'Jack3Coordinates' : {'Type' : 'PyTango.DevString', 'Description' : 'jack3 coordination: x,y,z'},
                         'CenterCoordinates': {'Type' : 'PyTango.DevString', 'Description' : 'center coordination: x,y,z'}}
    
    #This is azimuth angle for BL22-CLAESS (Synchrotron ALBA) which is 45 degrees indeed  
    #for your beamline change it accordingly to your azimuth angle 
    cosAzimuth = 0.70710681665463704
    sinAzimuth = -0.70710674571845633

    def __init__(self, inst, props):
        PseudoMotorController.__init__(self, inst, props)
        #self._log.setLevel(logging.DEBUG)

        try:
            self.jack1 = [float(c) for c in props['Jack1Coordinates'].split(',')]
            self.jack2 = [float(c) for c in props['Jack2Coordinates'].split(',')]
            self.jack3 = [float(c) for c in props['Jack3Coordinates'].split(',')]
            self.center = [float(c) for c in props['CenterCoordinates'].split(',')]
            
            if len(self.jack1) != 3 or len(self.jack2) != 3 or len(self.jack3) != 3:
                raise ValueError("Jack1, Jack2, Jack3 and Center Coordinates properties must be x,y,z coordinates in global system")
            
            if not (self. jack1[2] == self.jack2[2] == self.jack3[2]):
                raise ValueError('The mirror must be initially horizontal!')
            
        except ValueError, e:
            self._log.error('Could not parse class properties to generate coordinates.')
            raise e  

        #self.jackToMirrorInvariant = self.center[2] - self.jack1[2]
        
        # jacks in local virgin system, where point (0,0,0) is a center of a optical surface
        self.jack1local = [ji - ci for ji, ci in zip(self.jack1, self.center)]
        self.jack2local = [ji - ci for ji, ci in zip(self.jack2, self.center)]
        self.jack3local = [ji - ci for ji, ci in zip(self.jack3, self.center)]

        #rotating the table on z axis over azimuth angle 
        for jl in [self.jack1local, self.jack2local, self.jack3local]:
            jl[0], jl[1] = rotate_z(jl[0], jl[1], self.cosAzimuth, self.sinAzimuth)

        #self.zOffset = 1400 - self.jack1[2]
            
        self._log.debug("jack1local: %s" %repr(self.jack1local))
        self._log.debug("jack2local: %s" %repr(self.jack2local))
        self._log.debug("jack3local: %s" %repr(self.jack3local))

    def calc_physical(self, index, pseudos):
        return self.calc_all_physical(pseudos)[index - 1]

    def calc_pseudo(self, index, physicals):
        return self.calc_all_pseudo(physicals)[index - 1]

    def calc_all_physical(self, pseudos):
        z, pitch, roll = pseudos
        # Ax + By + Cz = D in local system:
        pitch = pitch / 1000
        roll = roll / 1000

        A, B, C = 0.0, 0.0, 1.0

        if roll != 0:
            cosRoll = math.cos(roll)
            sinRoll = math.sin(roll)
            A, C = rotate_y(A, C, cosRoll, sinRoll)
        if pitch != 0:
            cosPitch = math.cos(pitch)
            sinPitch = math.sin(pitch)
            B, C = rotate_x(B, C, cosPitch, sinPitch)

        #D of optical plane = 0 because (0, 0, 0) belongs to it
        D = 0
        #D of balls plane:
        #D -= self.jackToMirrorInvariant * (A ** 2 + B ** 2 + C ** 2) ** 0.5
        #but because rotations are unitary (A ** 2 + B ** 2 + C ** 2) = 1 and:
        #D -= self.jackToMirrorInvariant
        #D -= (self.center[2] - z)

        self._log.debug("Plane equation in local system A: %f, B: %f, C: %f, D: %f" % (A,B,C,D))

        jack1_local = (D - A * self.jack1local[0] - B * self.jack1local[1]) / C 
        jack2_local = (D - A * self.jack2local[0] - B * self.jack2local[1]) / C
        jack3_local = (D - A * self.jack3local[0] - B * self.jack3local[1]) / C

        self._log.debug("jack1_local: %s" %repr(jack1_local))
        self._log.debug("jack2_local: %s" %repr(jack2_local))
        self._log.debug("jack3_local: %s" %repr(jack3_local))

        jack1 = jack1_local + z #+ self.zOffset
        jack2 = jack2_local + z #+ self.zOffset
        jack3 = jack3_local + z #+ self.zOffset

        return jack1, jack2, jack3 

    def calc_all_pseudo(self, physicals):
        jack1, jack2, jack3 = physicals

#      Ax + By + Cz = D in global system:
        A = (self.jack2[1] - self.jack1[1]) * (jack3 - jack1) - (self.jack3[1] - self.jack1[1]) * (jack2 - jack1)
        B = (self.jack3[0] - self.jack1[0]) * (jack2 - jack1) - (self.jack2[0] - self.jack1[0]) * (jack3 - jack1)
        C = (self.jack2[0] - self.jack1[0]) * (self.jack3[1] - self.jack1[1])  - (self.jack3[0] - self.jack1[0]) * (self.jack2[1] - self.jack1[1])

        self._log.debug(" A: %f, B: %f, C: %f" % (A,B,C))
        ABCNorm = (A ** 2 + B ** 2 + C ** 2) ** 0.5
        self._log.debug("ABCNorm: %f" % (ABCNorm))
        if C < 0:
            ABCNorm *= -1      # its normal looks upwards!
        A /= ABCNorm; B /= ABCNorm; C /= ABCNorm
        # D of balls plane
        D = A * self.jack1[0] + B * self.jack1[1] + C * jack1

        #jackToMirrorInvariant = self.center[2] - jack1
        #D += self.jackToMirrorInvariant# of optical plane
        #self._log.debug('$$$$$$$$$$$$$$$$zOffset = %f', self.zOffset)
        #self._log.debug('$$$$$$$$$$$$$$$$jackToMirrorInv = %f; center = %f', self.jackToMirrorInvariant, self.center[2])
        self._log.debug("Plane equation in global system A: %f, B: %f, C: %f, D: %f" % (A,B,C,D))
        # C is never 0, i.e. the normal to the optical element is never horizontal
        z = (D - A * self.center[0] - B * self.center[1]) / C
#        z -= self.zOffset
        
        self._log.debug("z: %f" % z)

        # A  and B in local system (C is unchanged):
        locA, locB = rotate_z(A, B, self.cosAzimuth, self.sinAzimuth)
        self._log.debug("A, B in local subsystem (rotated on z axiz) A: %f, locB: %f" % (locA, locB))
        tanRoll = locA / C
        roll = math.atan(tanRoll) 

        tanPitch = -locB / (locA * math.sin(roll) + C * math.cos(roll))
        pitch = math.atan(tanPitch) * 1000
        roll *= 1000
        return z, pitch, roll