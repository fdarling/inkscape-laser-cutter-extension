#!/usr/bin/env python

# for running standalone (not invoked by Inkscape)
import sys
sys.path.append('/usr/share/inkscape/extensions')

import inkex
import simplepath
import simplestyle
import simpletransform
import cubicsuperpath
import bezmisc

import abc
from datetime import datetime
import math
import os
import copy
import base64
from io import BytesIO
from PIL import Image, ImageOps

VERSION = "0.0.1"

################################################
## START OF CODE RIPPED FROM turnkeylaser.py !!!
################################################

# This code was taken from the following GitHub project:
# https://github.com/TurnkeyTyranny/laser-gcode-exporter-inkscape-plugin
# which is a fork of:
# https://github.com/ajfoul/thlaser-inkscape-plugin
# which in turn is a fork of:
# https://github.com/lansing-makers-network/DEPRECATED---thlaser-inkscape-plugin
# the original code was licensed under the GPL-2.0

class TurnkeyOptions:
    def __init__(self):
        self.biarc_tolerance = 0.1
        self.biarc_max_split_depth = 4
        self.min_arc_radius = 0.0005
turnkeyoptions = TurnkeyOptions()

STRAIGHT_TOLERANCE = 0.0001
STRAIGHT_DISTANCE_TOLERANCE = 0.0001

###
###        Point (x,y) operations
###
## Pretty much what it sounds like: defines some arithmetic functions that can be applied to points.
class P:
    def __init__(self, x, y=None):
        if not y==None:
            self.x, self.y = float(x), float(y)
        else:
            self.x, self.y = float(x[0]), float(x[1])
    def __add__(self, other): return P(self.x + other.x, self.y + other.y)
    def __sub__(self, other): return P(self.x - other.x, self.y - other.y)
    def __neg__(self): return P(-self.x, -self.y)
    def __mul__(self, other):
        if isinstance(other, P):
            return self.x * other.x + self.y * other.y
        return P(self.x * other, self.y * other)
    __rmul__ = __mul__
    def __div__(self, other): return P(self.x / other, self.y / other)
    def mag(self): return math.hypot(self.x, self.y)
    def unit(self):
        h = self.mag()
        if h: return self / h
        else: return P(0,0)
    def dot(self, other): return self.x * other.x + self.y * other.y
    def rot(self, theta):
        c = math.cos(theta)
        s = math.sin(theta)
        return P(self.x * c - self.y * s,  self.x * s + self.y * c)
    def angle(self): return math.atan2(self.y, self.x)
    def __repr__(self): return '%f,%f' % (self.x, self.y)
    def pr(self): return "%.2f,%.2f" % (self.x, self.y)
    def to_list(self): return [self.x, self.y]


###
###        Functions to operate with CubicSuperPath
###

def csp_at_t(sp1,sp2,t):
    bez = (sp1[1][:],sp1[2][:],sp2[0][:],sp2[1][:])
    return     bezmisc.bezierpointatt(bez,t)

def cspbezsplit(sp1, sp2, t = 0.5):
    s1,s2 = bezmisc.beziersplitatt((sp1[1],sp1[2],sp2[0],sp2[1]),t)
    return [ [sp1[0][:], sp1[1][:], list(s1[1])], [list(s1[2]), list(s1[3]), list(s2[1])], [list(s2[2]), sp2[1][:], sp2[2][:]] ]

def cspbezsplitatlength(sp1, sp2, l = 0.5, tolerance = 0.01):
    bez = (sp1[1][:],sp1[2][:],sp2[0][:],sp2[1][:])
    t = bezmisc.beziertatlength(bez, l, tolerance)
    return cspbezsplit(sp1, sp2, t)

def cspseglength(sp1,sp2, tolerance = 0.001):
    bez = (sp1[1][:],sp1[2][:],sp2[0][:],sp2[1][:])
    return bezmisc.bezierlength(bez, tolerance)

def csplength(csp):
    total = 0
    lengths = []
    for sp in csp:
        for i in xrange(1,len(sp)):
            l = cspseglength(sp[i-1],sp[i])
            lengths.append(l)
            total += l
    return lengths, total


###
###        Distance calculattion from point to arc
###

def between(c,x,y):
        return x-STRAIGHT_TOLERANCE<=c<=y+STRAIGHT_TOLERANCE or y-STRAIGHT_TOLERANCE<=c<=x+STRAIGHT_TOLERANCE

def distance_from_point_to_arc(p, arc):
    P0,P2,c,a = arc
    dist = None
    p = P(p)
    r = (P0-c).mag()
    if r>0 :
        i = c + (p-c).unit()*r
        alpha = ((i-c).angle() - (P0-c).angle())
        if a*alpha<0:
            if alpha>0:    alpha = alpha-2*math.pi
            else: alpha = 2*math.pi+alpha
        if between(alpha,0,a) or min(abs(alpha),abs(alpha-a))<STRAIGHT_TOLERANCE :
            return (p-i).mag(), [i.x, i.y]
        else :
            d1, d2 = (p-P0).mag(), (p-P2).mag()
            if d1<d2 :
                return (d1, [P0.x,P0.y])
            else :
                return (d2, [P2.x,P2.y])

def get_distance_from_csp_to_arc(sp1,sp2, arc1, arc2, tolerance = 0.001 ): # arc = [start,end,center,alpha]
    n, i = 10, 0
    d, d1, dl = (0,(0,0)), (0,(0,0)), 0
    while i<1 or (abs(d1[0]-dl[0])>tolerance and i<2):
        i += 1
        dl = d1*1
        for j in range(n+1):
            t = float(j)/n
            p = csp_at_t(sp1,sp2,t)
            d = min(distance_from_point_to_arc(p,arc1), distance_from_point_to_arc(p,arc2))
            d1 = max(d1,d)
        n=n*2
    return d1[0]

################################################################################
###
###        Biarc function
###
###        Calculates biarc approximation of cubic super path segment
###        splits segment if needed or approximates it with straight line
###
################################################################################


def biarc(sp1, sp2, z1, z2, depth=0,):
    def biarc_split(sp1,sp2, z1, z2, depth):
        if depth<turnkeyoptions.biarc_max_split_depth:
            sp1,sp2,sp3 = cspbezsplit(sp1,sp2)
            l1, l2 = cspseglength(sp1,sp2), cspseglength(sp2,sp3)
            if l1+l2 == 0 : zm = z1
            else : zm = z1+(z2-z1)*l1/(l1+l2)
            return biarc(sp1,sp2,depth+1,z1,zm)+biarc(sp2,sp3,depth+1,z1,zm)
        else: return [ [sp1[1],'line', 0, 0, sp2[1], [z1,z2]] ]

    P0, P4 = P(sp1[1]), P(sp2[1])
    TS, TE, v = (P(sp1[2])-P0), -(P(sp2[0])-P4), P0 - P4
    tsa, tea, va = TS.angle(), TE.angle(), v.angle()
    if TE.mag()<STRAIGHT_DISTANCE_TOLERANCE and TS.mag()<STRAIGHT_DISTANCE_TOLERANCE:
        # Both tangents are zerro - line straight
        return [ [sp1[1],'line', 0, 0, sp2[1], [z1,z2]] ]
    if TE.mag() < STRAIGHT_DISTANCE_TOLERANCE:
        TE = -(TS+v).unit()
        r = TS.mag()/v.mag()*2
    elif TS.mag() < STRAIGHT_DISTANCE_TOLERANCE:
        TS = -(TE+v).unit()
        r = 1/( TE.mag()/v.mag()*2 )
    else:
        r=TS.mag()/TE.mag()
    TS, TE = TS.unit(), TE.unit()
    tang_are_parallel = ((tsa-tea)%math.pi<STRAIGHT_TOLERANCE or math.pi-(tsa-tea)%math.pi<STRAIGHT_TOLERANCE )
    if ( tang_are_parallel  and
                ((v.mag()<STRAIGHT_DISTANCE_TOLERANCE or TE.mag()<STRAIGHT_DISTANCE_TOLERANCE or TS.mag()<STRAIGHT_DISTANCE_TOLERANCE) or
                    1-abs(TS*v/(TS.mag()*v.mag()))<STRAIGHT_TOLERANCE)    ):
                # Both tangents are parallel and start and end are the same - line straight
                # or one of tangents still smaller then tollerance

                # Both tangents and v are parallel - line straight
        return [ [sp1[1],'line', 0, 0, sp2[1], [z1,z2]] ]

    c,b,a = v*v, 2*v*(r*TS+TE), 2*r*(TS*TE-1)
    if v.mag()==0:
        return biarc_split(sp1, sp2, z1, z2, depth)
    asmall, bsmall, csmall = abs(a)<10**-10,abs(b)<10**-10,abs(c)<10**-10
    if         asmall and b!=0:    beta = -c/b
    elif     csmall and a!=0:    beta = -b/a
    elif not asmall:
        discr = b*b-4*a*c
        if discr < 0:    raise ValueError, (a,b,c,discr)
        disq = discr**.5
        beta1 = (-b - disq) / 2 / a
        beta2 = (-b + disq) / 2 / a
        if beta1*beta2 > 0 :    raise ValueError, (a,b,c,disq,beta1,beta2)
        beta = max(beta1, beta2)
    elif    asmall and bsmall:
        return biarc_split(sp1, sp2, z1, z2, depth)
    alpha = beta * r
    ab = alpha + beta
    P1 = P0 + alpha * TS
    P3 = P4 - beta * TE
    P2 = (beta / ab)  * P1 + (alpha / ab) * P3

    def calculate_arc_params(P0,P1,P2):
        D = (P0+P2)/2
        if (D-P1).mag()==0: return None, None
        R = D - ( (D-P0).mag()**2/(D-P1).mag() )*(P1-D).unit()
        p0a, p1a, p2a = (P0-R).angle()%(2*math.pi), (P1-R).angle()%(2*math.pi), (P2-R).angle()%(2*math.pi)
        alpha =  (p2a - p0a) % (2*math.pi)
        if (p0a<p2a and  (p1a<p0a or p2a<p1a))    or    (p2a<p1a<p0a) :
            alpha = -2*math.pi+alpha
        if abs(R.x)>1000000 or abs(R.y)>1000000  or (R-P0).mag<turnkeyoptions.min_arc_radius :
            return None, None
        else :
            return  R, alpha
    R1,a1 = calculate_arc_params(P0,P1,P2)
    R2,a2 = calculate_arc_params(P2,P3,P4)
    if R1==None or R2==None or (R1-P0).mag()<STRAIGHT_TOLERANCE or (R2-P2).mag()<STRAIGHT_TOLERANCE    : return [ [sp1[1],'line', 0, 0, sp2[1], [z1,z2]] ]

    d = get_distance_from_csp_to_arc(sp1,sp2, [P0,P2,R1,a1],[P2,P4,R2,a2])
    if d > turnkeyoptions.biarc_tolerance and depth<turnkeyoptions.biarc_max_split_depth     : return biarc_split(sp1, sp2, z1, z2, depth)
    else:
        if R2.mag()*a2 == 0 : zm = z2
        else : zm  = z1 + (z2-z1)*(R1.mag()*a1)/(R2.mag()*a2+R1.mag()*a1)
        return [    [ sp1[1], 'arc', [R1.x,R1.y], a1, [P2.x,P2.y], [z1,zm] ], [ [P2.x,P2.y], 'arc', [R2.x,R2.y], a2, [P4.x,P4.y], [zm,z2] ]        ]

################################################
## END OF CODE RIPPED FROM turnkeylaser.py !!!
################################################

def floatToString(inputValue):
    result = "{0:.4f}".format(inputValue).rstrip('0').rstrip('.')
    # result = ('%.5f' % float(inputValue)).rstrip('0').rstrip('.')
    if result == '-0':
        return '0'
    return result

class GcodeModalNumericParameter:
    def __init__(self, paramLetter, initialValue):
        self.letter = str(paramLetter)
        self.value = float(initialValue)
        self.lastUsedValue = None

    def set(self, newValue):
        self.value = float(newValue)

    def get(self):
        return self.value

    def invalidate(self):
        self.lastUsedValue = None

    # produce a blank string if we would just repeat the last used value
    def use(self):
        result = ""
        if self.lastUsedValue != self.value:
            self.lastUsedValue = self.value
            result = self.letter + floatToString(self.value)
        return result

class AbstractPost:
    __metaclass__ = abc.ABCMeta
    @abc.abstractmethod
    def __init__(self, units = 'mm'): pass
    @abc.abstractmethod
    def begin(self, outFilePath): pass
    @abc.abstractmethod
    def end(self): pass
    @abc.abstractmethod
    def setRapidFeedrate(self, feedrate): pass
    @abc.abstractmethod
    def setFeedrate(self, feedrate): pass
    @abc.abstractmethod
    def setPowerRatio(self, powerRatio): pass
    @abc.abstractmethod
    def writeComment(self, comment): pass
    @abc.abstractmethod
    def home(self): pass
    @abc.abstractmethod
    def moveTo(self, pos): pass
    @abc.abstractmethod
    def lineTo(self, pos): pass
    @abc.abstractmethod
    def arcTo(self, endPos, relCenterPos, clockwise = True): pass

class GenericPost(AbstractPost):
    __metaclass__ = abc.ABCMeta
    def __init__(self, units = 'mm'):
        self.motiongcode = GcodeModalNumericParameter('G', 0)
        self.posx = GcodeModalNumericParameter('X', 0.0)
        self.posy = GcodeModalNumericParameter('Y', 0.0)
        self.posi = GcodeModalNumericParameter('I', 0.0)
        self.posj = GcodeModalNumericParameter('J', 0.0)
        self.rapidFeedrate = GcodeModalNumericParameter('F', 2000.0)
        self.feedrate = GcodeModalNumericParameter('F', 300.0)
        # TODO raise an error, don't just default to mm
        self.units = 'mm'
        if units == 'in':
            self.units = 'in'
        self.outFile = None

    def begin(self, outFilePath):
        self.outFile = open(outFilePath, "w")
        # TODO move this somewhere else to make it apply to all posts
        self.writeComment(" G-code file generated on %s" % (datetime.now().isoformat())) # .strftime("%Y-%m-%d %H:%M:%S %Z")
        self.outFile.write("G90 ; absolute coordinates\n")
        if self.units == 'in':
            self.outFile.write("G20 ; units = in\n")
        else: # default to mm
            self.outFile.write("G21 ; units = mm\n")
        self.outFile.write("G17 ; XY plane for arcs\n")
        # self.outFile.write("G64 P0.01 ; allow a small amount of path blending\n")

    def end(self):
        pass
        # self.outFile.write("M84 ; disable stepper motors\n")

    def setRapidFeedrate(self, feedrate):
        self.rapidFeedrate.set(feedrate)

    def setFeedrate(self, feedrate):
        self.feedrate.set(feedrate)

    # @abc.abstractmethod
    # def setPowerRatio(self, powerRatio): pass

    def writeComment(self, comment):
        comment = comment.replace("\r\n", "\n")
        comment = comment.replace("\r", "\n")
        comment = comment.replace("\n", "; ")
        self.outFile.write("; %s\n" % (comment))

    def _WriteLine(self, *paramList):
        # put non-blank parameter/value as a line with spaces between
        line = " ".join(filter(None, map(lambda x: x.use(), paramList)))
        # output the line only if it has something new to say
        if line:
            self.outFile.write(line + "\n")

    def home(self):
        # NOTE: Marlin ignores coordinate parameters, but CAMotics and LinuxCNC (EMC2) use them
        self.outFile.write("G28 X0 Y0 ; home X/Y\n")
        self.motiongcode.invalidate()

    def moveTo(self, pos):
        # this avoid emitting "G0" on a line by itself because we don't actually move yet
        if self.posx.get() == pos[0] and self.posy.get() == pos[1]:
            return
        self.posx.set(pos[0])
        self.posy.set(pos[1])
        self.motiongcode.set(0) # G0 = rapid move
        self._WriteLine(self.motiongcode, self.posx, self.posy, self.rapidFeedrate)

    def _LineTo(self, pos, *extraParams):
        # this avoid emitting "G1" on a line by itself because we don't actually move yet
        if self.posx.get() == pos[0] and self.posy.get() == pos[1]:
            return
        self.posx.set(pos[0])
        self.posy.set(pos[1])
        self.motiongcode.set(1) # G1 = linear cutting move
        self._WriteLine(self.motiongcode, self.posx, self.posy, *extraParams)

    def lineTo(self, pos):
        self._LineTo(pos, self.feedrate)

    def _ArcTo(self, endPos, relCenterPos, clockwise, *extraParams):
        arcType = 2
        if not clockwise:
            arcType = 3
        self.motiongcode.set(arcType) # G2/G3 = arc segment cutting move
        self.posx.set(endPos[0])
        self.posy.set(endPos[1])
        self.posi.set(relCenterPos[0])
        self.posj.set(relCenterPos[1])
        self._WriteLine(self.motiongcode, self.posx, self.posy, self.posi, self.posj, *extraParams)
        # NOTE: we don't trust I/J between arc calls because they are relative coordinates
        self.posi.invalidate()
        self.posj.invalidate()

    def arcTo(self, endPos, relCenterPos, clockwise = True):
        self._ArcTo(endPos, relCenterPos, clockwise, self.feedrate)

class GRBLPost(GenericPost):
    SPINDLE_MIN = 0.0
    SPINDLE_MAX = 100.0

    def __init__(self, units = 'mm'):
        GenericPost.__init__(self, units)
        self.spindleSpeed = GcodeModalNumericParameter('S', GRBLPost.SPINDLE_MAX)
        self.spindleDirection = GcodeModalNumericParameter('M', 5) # M5 = stop spindle

    def setPowerRatio(self, powerRatio):
        powerRatio = float(powerRatio)
        if powerRatio < 0.0:
            powerRatio = 0.0
        elif powerRatio > 1.0:
            powerRatio = 1.0

        self.spindleSpeed.set(GRBLPost.SPINDLE_MIN + float(powerRatio)*(GRBLPost.SPINDLE_MAX - GRBLPost.SPINDLE_MIN))

    def _WriteSpindleLine(self, enabled):
        if enabled:
            self.spindleDirection.set(3) # M3 = spindle on, clockwise
            self._WriteLine(self.spindleDirection, self.spindleSpeed)
        else:
            self.spindleDirection.set(5) # M5 = spindle off
            self._WriteLine(self.spindleDirection)

    def end(self):
        self._WriteSpindleLine(False)
        self._WriteLine(self.spindleDirection)
        self.outFile.write("M30 ; end program\n")

    def home(self):
        self._WriteSpindleLine(False)
        GenericPost.home(self)

    def moveTo(self, pos):
        if self.posx.get() == pos[0] and self.posy.get() == pos[1]:
            return
        self._WriteSpindleLine(False)
        GenericPost.moveTo(self, pos)

    def lineTo(self, pos):
        if self.posx.get() == pos[0] and self.posy.get() == pos[1]:
            return
        self._WriteSpindleLine(True)
        self._LineTo(pos, self.feedrate)

    def arcTo(self, endPos, relCenterPos, clockwise = True):
        self._WriteSpindleLine(True)
        self._ArcTo(endPos, relCenterPos, clockwise, self.feedrate)

class SmoothiewarePost(GenericPost):
    def __init__(self, units = 'mm'):
        GenericPost.__init__(self, units)
        self.powerRatio = GcodeModalNumericParameter('S', 1.0)

    def setPowerRatio(self, powerRatio):
        powerRatio = float(powerRatio)
        if powerRatio < 0.0:
            powerRatio = 0.0
        elif powerRatio > 1.0:
            powerRatio = 1.0
        self.powerRatio.set(powerRatio)

    def lineTo(self, pos):
        self._LineTo(pos, self.powerRatio, self.feedrate)

    def arcTo(self, endPos, relCenterPos, clockwise = True):
        self._ArcTo(endPos, relCenterPos, clockwise, self.powerRatio, self.feedrate)

class LaserLayer:
    def __init__(self, label, defaultSettings, skipName = False):
        self.label = label
        self.rapid = defaultSettings['rapidFeedrate']
        self.feedrate = defaultSettings['feedrate']
        self.power = defaultSettings['power']
        self.dpi = defaultSettings['DPI']
        self.passes = defaultSettings['passes']
        self.paths = []

        # we have the option to skip parsing the name
        if skipName:
            return

        LAYER_OPTIONS = { \
            'rapid':  {'target': 'rapid',    'type': float, 'min': 0.0, 'max': None, 'min_inclusive': False}, \
            'feed':   {'target': 'feedrate', 'type': float, 'min': 0.0, 'max': None, 'min_inclusive': False}, \
            'dpi':    {'target': 'dpi',      'type': float, 'min': 0.0, 'max': None, 'min_inclusive': False}, \
            'passes': {'target': 'passes',   'type': int  , 'min': 0  , 'max': None, 'min_inclusive': True }, \
        }

        # locate the options list
        leftBracketIndex  = label.find('[')
        rightBracketIndex = label.find(']')
        # parse the power level that is before the options list
        powerString = ''
        if leftBracketIndex == -1:
            powerString = label
        else:
            powerString = label[:leftBracketIndex]
        try:
            self.power = float(powerString)/100.0
            if self.power < 0.0:
                self.power = -1.0
                raise ValueError()
        except:
            inkex.errormsg("WARNING: For layer \"%s\" unable to parse the beginning of the name as a power level percentage (0-100). Note that we don't actually want a percent sign in there! As a result, it will be cut with the default power level." % (label))
        # warn about unmatched square brackets
        # TODO: check for nested brackets and warn about it
        if (leftBracketIndex == -1) != (rightBracketIndex == -1):
            inkex.errormsg("WARNING: a layer name has unmatched square brackets [], unable to parse it!\nlayer name = \"" + label + "\"")
        # if we don't have opening/closing square brackets, bail early
        if leftBracketIndex == -1 or rightBracketIndex == -1:
            return
        # extract the text between the brackets
        optionsString = label[leftBracketIndex+1:rightBracketIndex].strip()
        # interpret it as a comma separated parameter list
        options = optionsString.split(',')
        for option in options:
            # separate the parameter name from the value it's being set to
            words = option.split('=')
            if len(words) != 2:
                inkex.errormsg("WARNING: For layer \"%s\" couldn't parse option \"%s\" as it isn't in the format of option=value." % (label, option))
                continue
            # normalize the left/right side
            leftWord = words[0].strip().lower()
            rightWord = words[1].strip()
            # make sure that neither side of the assignment is empty
            if not leftWord or not rightWord:
                inkex.errormsg("WARNING: For layer \"%s\" couldn't parse option \"%s\" as it isn't in the format of option=value." % (label, option))
                continue
            # look up the parameter we're setting
            optionCharacteristics = LAYER_OPTIONS.get(leftWord)
            if not optionCharacteristics:
                inkex.errormsg("WARNING: For layer \"%s\" unrecognized option \"%s\"." % (label, leftWord))
                continue
            # attempt casting the value to the parameter's expected type
            value = None
            try:
                value = optionCharacteristics['type'](rightWord)
            except ValueError:
                inkex.errormsg("WARNING: For layer \"%s\" ignoring value \"%s\" for option \"%s\" as it cannot be interpreted as the type \"%s\"." % (label, rightWord, leftWord, optionCharacteristics['type']))
                continue
            # make sure the value range is valid
            if optionCharacteristics['min'] != None and (value < optionCharacteristics['min'] or (value == optionCharacteristics['min'] and not optionCharacteristics['min_inclusive'])):
                relationship = "greater than"
                if optionCharacteristics['min_inclusive']:
                    relationship = "greater than or equal to"
                inkex.errormsg("WARNING: For layer \"%s\" ignoring value \"%s\" for option \"%s\" as it needs to be %s %s." % (label, rightWord, leftWord, relationship, optionCharacteristics['min']))
                continue
            # TODO test checking against the max value, currently nothing uses this...
            elif optionCharacteristics['max'] != None and value > optionCharacteristics['max']:
                inkex.errormsg("WARNING: For layer \"%s\" ignoring value \"%s\" for option \"%s\" as it needs to be less than or equal to %s." % (label, rightWord, leftWord, optionCharacteristics['max']))
                continue
            # if it's passes all the requirements, actually use the value
            setattr(self, optionCharacteristics['target'], value)

class LaserCutterGcodeExportEffect(inkex.Effect):
    MM_PER_INCH = 25.4

    def __init__(self):
        inkex.Effect.__init__(self)
        self.OptionParser.add_option("-d", "--directory",   action="store", type="string",  dest="directory",  default="",         help="Output directory")
        self.OptionParser.add_option("-f", "--filename",    action="store", type="string",  dest="filename",   default="",         help="Output file name")
        self.OptionParser.add_option("",   "--dpi",         action="store", type="float",   dest="dpi",        default="300.0",    help="Default DPI (dots per inch) for rastering")
        self.OptionParser.add_option("-m", "--Mfeed",       action="store", type="float",   dest="rapidFeed",  default="2000.0",   help="Default rapid feedrate in units/minute")
        self.OptionParser.add_option("-p", "--feed",        action="store", type="float",   dest="feedrate",   default="300.0",    help="Default cut feedrate in units/minute")
        self.OptionParser.add_option("-l", "--laser",       action="store", type="float",   dest="power",      default="10.0",     help="Default laser intensity (0-100%)")
        self.OptionParser.add_option("",   "--passes",      action="store", type="int",     dest="passes",     default="1",        help="Default laser passes")
        self.OptionParser.add_option("",   "--homebefore",  action="store", type="inkbool", dest="homeBefore", default=True,       help="Home X/Y at beginning of job")
        self.OptionParser.add_option("",   "--homeafter",   action="store", type="inkbool", dest="homeAfter",  default=True,       help="Home X/Y at end of job")
        self.OptionParser.add_option("",   "--mainboard",   action="store", type="string",  dest="mainboard",  default="smoothie", help="Mainboard")
        self.OptionParser.add_option("",   "--unit",        action="store", type="string",  dest="units",      default="mm",       help="Units")
        self.OptionParser.add_option("",   "--spindle-min", action="store", type="float",   dest="spindleMin", default="0.0",      help="Spindle minimum RPM for 0% laser power")
        self.OptionParser.add_option("",   "--spindle-max", action="store", type="float",   dest="spindleMax", default="100.0",    help="Spindle maximum RPM for 100% laser power")
        self.OptionParser.add_option("",   "--draw-curves", action="store", type="inkbool", dest="drawCurves", default=False,      help="Draws curves to show what geometry was processed")
        self.OptionParser.add_option("",   "--tab",         action="store", type="string",  dest="tab",        default="",         help="stub, required to supress an undefined option error")

        self.TAG_HANDLERS = { \
            inkex.addNS('svg' , 'svg'): self._ProcessStub, \
            inkex.addNS('g'   , 'svg'): self._ProcessStub, \
            inkex.addNS('circle', 'svg'): self._ProcessCircle, \
            inkex.addNS('ellipse', 'svg'): self._ProcessEllipse, \
            inkex.addNS('image', 'svg'): self._ProcessImage, \
            inkex.addNS('line', 'svg'): self._ProcessLine, \
            inkex.addNS('path', 'svg'): self._ProcessPath, \
            inkex.addNS('polygon', 'svg'): self._ProcessPolygon, \
            inkex.addNS('polyline', 'svg'): self._ProcessPolyline, \
            inkex.addNS('rect', 'svg'): self._ProcessRect, \
            # inkex.addNS('text', 'svg'): self., \
            # inkex.addNS('use', 'svg'): self., \
        }
        self.TAG_IGNORE_LIST = { \
            inkex.addNS('defs', 'svg'),
            inkex.addNS('namedview', 'sodipodi'),
            inkex.addNS('metadata', 'svg'),
        }

        self.laserLayers = []
        self.outputLayer = None
        self.filePath = None
        self.inkscapeDPI = 90.0 # TODO: support 96.0 DPI for Inkscape >= 0.92?
        self.scaleFactor = self.MM_PER_INCH/self.inkscapeDPI
        # NOTE: this should probably match the extension's ".inx" file for consistency
        self.defaultLayerSettings = { \
            'rapidFeed': 2000.0, \
            'feedrate': 300.0, \
            'power': 0.1, \
            'DPI': 300.0 \
        }

    def _FixedDirectory(self, directory):
        DEFAULT_DIRECTORY = '~/Desktop'
        return os.path.expanduser(directory or DEFAULT_DIRECTORY)

    def _FixedFilename(self, filename):
        KNOWN_EXTENSIONS = ['g', 'gcode', 'ngc', 'tap', 'nc']
        DEFAULT_FILENAME = 'output'
        DEFAULT_EXTENSION = '.g'
        if not filename:
            filename = DEFAULT_FILENAME
        parts = filename.lower().split('.')
        if len(parts) == 1 or (parts[-1] not in KNOWN_EXTENSIONS):
            filename += '.g'
        return filename

    def _AddPath(self, pathString, transformMatrix, laserLayer, objectId):
        # cubicsuperpath.parsePath() crashes on an empty string as input, upstream bug!
        if not pathString.strip():
            return
        # convert the various types of SVG path operations into a more standardized form, losing path closed/openness (we don't care though)
        parsedPathCubic = cubicsuperpath.parsePath(pathString)
        # "bake in" the transformation to the path data
        simpletransform.applyTransformToPath(transformMatrix, parsedPathCubic)
        # save the transformed path
        laserLayer.paths.append({'id': objectId, 'path': parsedPathCubic})

    def _ProcessImage(self, elt, transformMatrix, laserLayer):
        eltX, eltY, eltW, eltH = 0.0, 0.0, 0.0, 0.0
        try:
            eltX = float(elt.get("x", "0.0"))
            eltY = float(elt.get("y", "0.0"))
            # TODO support using the image size by default if it's unspecified
            eltW = float(elt.get("width", "0.0"))
            eltH = float(elt.get("height", "0.0"))
        except:
            inkex.errormsg("WARNING: skipping <image> element as at least one of it's 'x', 'y', 'width', or 'height' attributes is invalid: id='" + str(elt.get("id")) + "'")
            return
        # TODO support non-embedded images (files outside of the SVG itself)
        imageStr = str((elt.get(inkex.addNS('href', 'xlink')))).replace("data:image/png;base64,","").replace("data:image/jpeg;base64,","")
        img = Image.open(BytesIO(base64.b64decode(imageStr)))
        # make sure that any pixels hidden due to an alpha channel become white *not* black after converting to grayscale
        if img.mode == 'RGBA':
            img = Image.alpha_composite(Image.new('RGBA', img.size, 'white'), img)
        # convert to 8-bit grayscale
        if img.format != 'L':
            img = img.convert('L')
        # we invert it so that black (a value of 0) and white (a value of 255) get swapped, so that black becomes non-zero and "true" for cutting purposes
        img = ImageOps.invert(img)
        # make sure it's not a zero area image
        if img.size[0] <= 0 or img.size[1] <= 0:
            return
        # calculate the effective edge length of the image as it's displayed, and use this to determine how many raster rows will be needed
        p1, p2 = [0.0, 0.0], [0.0, eltH]
        simpletransform.applyTransformToPoint(transformMatrix, p1)
        simpletransform.applyTransformToPoint(transformMatrix, p2)
        transformedHeight = abs(p2[1] - p1[1])
        # actual row spacing = (1/DPI) = (inches per px [i/px])*(transformed height [px])/((image data rows)*scaleFactor)
        # DPI = ((image data rows) * scaleFactor)/((inches per px) * (transformed height)
        # scaleFactor = DPI*(transformed height)*(inches per px)/(image data rows)
        scaleFactor = laserLayer.dpi*transformedHeight*(1.0/self.inkscapeDPI)/float(img.size[1]) # NOTE: was hardcoded with 96.0 *not* 90.0 even though this was pre-Inkscape-v0.92 ???
        # TODO: optimize this to not need to resize the image, instead change how we iterate over it (manually resample it in other words)
        img = img.resize((img.size[0], int(float(img.size[1])*scaleFactor)))
        pix = img.load()
        # paint the image in small lines
        newPathString = ""
        for row in range(img.size[1]):
            # map the image row to the specified rendered-image coordinate system
            y = eltY + eltH*float(row)/float(img.size[1])
            x = eltX
            startCol = -1
            col = 0
            # alternate the direction of iteration every row
            if row & 1:
                col = img.size[0] - 1
            # process the entire row for lasing
            while col >= 0 and col < img.size[0]:
                # alternate the direction of iteration every row
                if row & 1:
                    col_next = col - 1
                else:
                    col_next = col + 1
                # grab the current pixel color
                color = pix[col, row]
                # grab the next pixel color
                color_next = 0
                if col_next >= 0 and col_next < img.size[0]:
                    color_next = pix[col_next, row]
                # if we were between solid regions, start a new run length by remembering the beginning column
                if color and startCol == -1:
                    startCol = col
                # if we're finishing a run of solid pixels...
                if startCol != -1 and not color_next:
                    # logic due to alternating directions
                    col1 = startCol
                    col2 = col_next
                    if row & 1:
                        col1 = startCol + 1
                        col2 = col
                    # map the image coordinates to the <image> tag's scaling
                    x1 = eltX + eltW*float(col1)/float(img.size[0])
                    x2 = eltX + eltW*float(col2)/float(img.size[0])
                    newPathString += " M %.5f,%.5f" % (x1, y)
                    newPathString += " L %.5f,%.5f" % (x2, y)
                    # we've ended this current run of opaque pixels
                    startCol = -1
                # advance to the next pixel
                col = col_next
        self._AddPath(newPathString, transformMatrix, laserLayer, elt.get("id"))

    def _ProcessStub(self, polyline, transformMatrix, laserLayer):
        pass

    def _ProcessLine(self, line, transformMatrix, laserLayer):
        x1, y1, x2, y2 = 0.0, 0.0, 0.0, 0.0
        try:
            x1 = float(line.get("x1", "0.0"))
            y1 = float(line.get("y1", "0.0"))
            x2 = float(line.get("x2", "0.0"))
            y2 = float(line.get("y2", "0.0"))
        except:
            inkex.errormsg("WARNING: skipping <line> element as at least one of it's 'x1', 'y1', 'x2', or 'y2' attributes is invalid: id='" + str(line.get("id")) + "'")
            return
        # build path string
        newPathString =   "M %.5f,%.5f" % (x1, y1)
        newPathString += " L %.5f,%.5f" % (x2, y2)
        self._AddPath(newPathString, transformMatrix, laserLayer, line.get("id"))

    def _ProcessPolyline(self, polyline, transformMatrix, laserLayer, closePath = False):
        # perhaps technically not treating commas and whitespace correctly... see https://www.w3.org/TR/SVG/shapes.html#PointsBNF
        pointsString = polyline.get("points", "").replace(',', ' ')
        # split at whitespace boundaries, consecutive whitespace are treated as one
        coordinateList = pointsString.split()
        # skip empty polylines
        if len(coordinateList) == 0:
            return
        # make sure we have coordinate pairs (even number of coordinates in the list)
        if len(coordinateList) & 1:
            inkex.errormsg("WARNING: skipping <polyline> element as it has an odd number X/Y values listed, which cannot make coordinate pairs: id='" + str(polyline.get("id")) + "'")
            return
        # build path string
        newPathString = "M"
        # remember the first point for potentially closing the path
        firstPoint = None
        try:
            # process the X/Y values in pairs
            for x, y in zip(*[iter(coordinateList)]*2):
                point = [float(x), float(y)]
                if firstPoint is None:
                    firstPoint = point
                # NOTE: we don't specify 'L' (line-to) after the initial 'M' (move-to) because SVG spec says it's implicit anyways
                newPathString += " %.5f,%.5f" % (point[0], point[1])
        except:
            inkex.errormsg("WARNING: skipping <polyline> element as it has at least one invalid coordinate in the 'points' attribute: id='" + str(polyline.get("id")) + "'")
            return
        # possibly close the path (used for polygons not polylines)
        if closePath and firstPoint:
            newPathString += " Z"
        self._AddPath(newPathString, transformMatrix, laserLayer, polyline.get("id"))

    def _ProcessPolygon(self, polygon, transformMatrix, laserLayer):
        self._ProcessPolyline(polygon, transformMatrix, laserLayer, True)

    def _ProcessRect(self, rect, transformMatrix, laserLayer):
        x, y, w, h, rx, ry = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        try:
            x = float(rect.get("x", "0.0"))
            y = float(rect.get("y", "0.0"))
            w = float(rect.get("width", "0.0"))
            h = float(rect.get("height", "0.0"))
            # TODO better error handling of specifying invalid rx/ry and also copying rx/ry
            rx = float(rect.get("rx", "0.0"))
            ry = float(rect.get("ry", "0.0"))
        except:
            inkex.errormsg("WARNING: skipping <rect> element as at least one of it's 'x', 'y', 'width', 'height', 'rx', or 'ry' attributes is invalid: id='" + str(rect.get("id")) + "'")
            return
        # build path string
        newPathString = ""
        # TODO test case where only one of them is zero...
        if rx == 0.0 and ry == 0.0:
            newPathString  =  "M %.5f,%.5f" % (x, y)
            newPathString += " L %.5f,%.5f" % (x + w, y)
            newPathString += " L %.5f,%.5f" % (x + w, y + h)
            newPathString += " L %.5f,%.5f" % (x, y + h)
            newPathString += " L %.5f,%.5f" % (x, y)
            newPathString += " Z"
        else:
            newPathString  =  "M %.5f,%.5f" % (x     + rx, y)
            newPathString += " L %.5f,%.5f" % (x + w - rx, y)
            newPathString += " A %.5f,%.5f %.5f 0 1 %.5f,%.5f" % (rx, ry, 0.0, \
                                               x + w     , y     + ry)
            newPathString += " L %.5f,%.5f" % (x + w     , y + h - ry)
            newPathString += " A %.5f,%.5f %.5f 0 1 %.5f,%.5f" % (rx, ry, 0.0, \
                                               x + w - rx, y + h)
            newPathString += " L %.5f,%.5f" % (x     + rx, y + h)
            newPathString += " A %.5f,%.5f %.5f 0 1 %.5f,%.5f" % (rx, ry, 0.0, \
                                               x         , y + h - ry)
            newPathString += " L %.5f,%.5f" % (x         , y     + ry)
            newPathString += " A %.5f,%.5f %.5f 0 1 %.5f,%.5f" % (rx, ry, 0.0, \
                                               x + rx    , y)
            newPathString += " Z"
        # process the rect as a generic path at this point
        self._AddPath(newPathString, transformMatrix, laserLayer, rect.get("id"))

    def _ProcessEllipse(self, ellipse, transformMatrix, laserLayer, wasCircle = False):
        cx, cy, rx, ry = 0.0, 0.0, 0.0, 0.0
        try:
            cx = float(ellipse.get("cx", "0.0"))
            cy = float(ellipse.get("cy", "0.0"))
            if wasCircle:
                rx = ry = float(ellipse.get("r", "0.0"))
            else:
                rx = float(ellipse.get("rx", "0.0"))
                ry = float(ellipse.get("ry", "0.0"))
        except:
            if wasCircle:
                inkex.errormsg("WARNING: skipping <circle> element as at least one of it's 'cx', 'cy', or 'r' attributes is invalid: id='" + str(ellipse.get("id")) + "'")
            else:
                inkex.errormsg("WARNING: skipping <ellipse> element as at least one of it's 'cx', 'cy', 'rx', or 'ry' attributes is invalid: id='" + str(ellipse.get("id")) + "'")
            return
        # SVG spec says we need positive values of rx/ry
        if rx <= 0.0 or ry <= 0.0:
            if wasCircle:
                inkex.errormsg("WARNING: skipping <circle> element as it's 'r' attribute is not a positive value: id='" + str(ellipse.get("id")) + "'")
            else:
                inkex.errormsg("WARNING: skipping <ellipse> element as at least one of it's 'rx' or 'ry' attributes is not a positive value: id='" + str(ellipse.get("id")) + "'")
            return
        # build path string
        newPathString  =  "M %.5f,%.5f" % (-rx + cx, cy)
        #(rx ry x-axis-rotation large-arc-flag sweep-flag x y)
        newPathString += " A %.5f,%.5f %.5f 0 1 %.5f,%.5f" % (rx, ry, 0.0, 0.0 + cx, -ry + cy)
        newPathString += " A %.5f,%.5f %.5f 0 1 %.5f,%.5f" % (rx, ry, 0.0,  rx + cx, 0.0 + cy)
        newPathString += " A %.5f,%.5f %.5f 0 1 %.5f,%.5f" % (rx, ry, 0.0, 0.0 + cx,  ry + cy)
        newPathString += " A %.5f,%.5f %.5f 0 1 %.5f,%.5f" % (rx, ry, 0.0, -rx + cx, 0.0 + cy)
        newPathString += " Z"
        # process the circle as a generic path at this point
        self._AddPath(newPathString, transformMatrix, laserLayer, ellipse.get("id"))

    def _ProcessCircle(self, circle, transformMatrix, laserLayer):
        self._ProcessEllipse(circle, transformMatrix, laserLayer, True)

    def _ProcessPath(self, path, transformMatrix, laserLayer):
        data = path.get("d")
        if not data:
            inkex.errormsg("WARNING: skipping <path> element missing it's 'd' attribute: id='" + str(path.get("id")) + "'")
            return
        self._AddPath(data, transformMatrix, laserLayer, path.get("id"))

    def _ProcessOperations(self, post, pageHeight):
        post.begin(self.filePath)
        if self.options.homeBefore:
            post.home()
        for layer in self.laserLayers:
            # skip empty layers
            if not layer.paths:
                continue
            # display layer information
            post.writeComment('****** LAYER \"' + layer.label + '" ******')
            post.writeComment('layer rapid  =  ' + floatToString(layer.rapid))
            post.writeComment('layer feed   =  ' + floatToString(layer.feedrate))
            post.writeComment('layer power  =  ' + floatToString(layer.power))
            post.writeComment('layer dpi    =  ' + floatToString(layer.dpi))
            post.writeComment('layer passes =  ' + str(layer.passes))
            # emit the G-code for each path
            for passIndex in range(layer.passes):
                if layer.passes > 1:
                    post.writeComment("pass %i/%i" % (passIndex + 1, layer.passes))
                for operation in layer.paths:
                    # set feedrates/power levels
                    post.setRapidFeedrate(layer.rapid)
                    post.setFeedrate(layer.feedrate)
                    post.setPowerRatio(layer.power)
                    # emit the actual transformed path (from 2D graphics origin at top left to be CNC origin at bottom left)
                    # TODO: make the origin a configurable option
                    transformedPath = copy.deepcopy(operation['path'])
                    scaleM = [[self.scaleFactor, 0.0, 0.0], [0.0, self.scaleFactor, 0.0]]
                    transM = [[1.0, 0.0, 0.0], [0.0, -1.0, pageHeight]]
                    transformMatrix = simpletransform.composeTransform(scaleM, transM)
                    simpletransform.applyTransformToPath(transformMatrix, transformedPath)
                    newPathParsed = []
                    for subpath in transformedPath:
                        newPathParsed.append([[subpath[0][1][0], subpath[0][1][1]], 'move', 0, 0])
                        for i in range(1,len(subpath)):
                            sp1 = [[subpath[i-1][j][0], subpath[i-1][j][1]] for j in range(3)]
                            sp2 = [[subpath[i  ][j][0], subpath[i  ][j][1]] for j in range(3)]
                            newPathParsed += biarc(sp1,sp2,0,0)
                        newPathParsed.append([[subpath[-1][1][0], subpath[-1][1][1]], 'end', 0, 0])
                    ###########################
                    idString = operation['id']
                    if not idString:
                        idString = ''
                    post.writeComment("path for object id=" + idString)
                    for i in range(1,len(newPathParsed)):
                        s, si = newPathParsed[i-1], newPathParsed[i]
                        #G00 : Move with the laser off to a new point
                        if s[1] == 'move':
                            post.moveTo((si[0][0], si[0][1]))
                        elif s[1] == 'end':
                            pass
                        #G01 : Move with the laser turned on to a new point
                        elif s[1] == 'line':
                            post.lineTo((si[0][0], si[0][1]))
                        #G02 and G03 : Move in an arc with the laser turned on.
                        elif s[1] == 'arc':
                            dx = s[2][0]-s[0][0]
                            dy = s[2][1]-s[0][1]
                            if abs((dx**2 + dy**2)) > turnkeyoptions.min_arc_radius:
                                r1 = P(s[0])-P(s[2])
                                r2 = P(si[0])-P(s[2])
                                post.arcTo((si[0][0], si[0][1]), (dx, dy), not (s[3] > 0))
                            #The arc is less than the minimum arc radius, draw it as a straight line.
                            else:
                                post.lineTo((si[0][0], si[0][1]))
                    ###########################
                    # optionally show the untransformed toolpath in Inkscape for debugging
                    if self.outputLayer is not None:
                        # PATH_STYLE = 'stroke:#FF0000;fill:none;stroke-width:0.1px;stroke-linecap:round;stroke-linejoin:round'
                        if 1: # show paths after applying bi-arc approximation
                            BIARC_STYLE = { \
                                'biarc0': simplestyle.formatStyle({ 'stroke': '#88f', 'fill': 'none', 'strokeWidth':'0.1' }), \
                                'biarc1': simplestyle.formatStyle({ 'stroke': '#8f8', 'fill': 'none', 'strokeWidth':'0.1' }), \
                                'line':   simplestyle.formatStyle({ 'stroke': '#f88', 'fill': 'none', 'strokeWidth':'0.1' }), \
                                'area':   simplestyle.formatStyle({ 'stroke': '#777', 'fill': 'none', 'strokeWidth':'0.1' }), \
                            }
                            newPathParsed_untransformed = []
                            for subpath in operation['path']:
                                newPathParsed_untransformed.append([[subpath[0][1][0], subpath[0][1][1]], 'move', 0, 0])
                                for i in range(1,len(subpath)):
                                    sp1 = [[subpath[i-1][j][0], subpath[i-1][j][1]] for j in range(3)]
                                    sp2 = [[subpath[i  ][j][0], subpath[i  ][j][1]] for j in range(3)]
                                    newPathParsed_untransformed += biarc(sp1,sp2,0,0)
                                newPathParsed_untransformed.append([[subpath[-1][1][0], subpath[-1][1][1]], 'end', 0, 0])
                            s, arcn = '', 0
                            for si in newPathParsed_untransformed:
                                if s!='':
                                    if s[1] == 'line':
                                        inkex.etree.SubElement(self.outputLayer, inkex.addNS('path', 'svg'), {'style': BIARC_STYLE['line'], 'd':'M %s,%s L %s,%s' % (s[0][0], s[0][1], si[0][0], si[0][1])})
                                    elif s[1] == 'arc':
                                        arcn += 1
                                        sp = s[0]
                                        c = s[2]
                                        a =  ( (P(si[0])-P(c)).angle() - (P(s[0])-P(c)).angle() )%(2*math.pi) #s[3]
                                        if s[3]*a<0:
                                                if a>0:    a = a-2*math.pi
                                                else: a = 2*math.pi+a
                                        r = math.sqrt( (sp[0]-c[0])**2 + (sp[1]-c[1])**2 )
                                        a_st = ( math.atan2(sp[0]-c[0],- (sp[1]-c[1])) - math.pi/2 ) % (math.pi*2)
                                        if a>0:
                                            a_end = a_st+a
                                        else:
                                            a_end = a_st*1
                                            a_st = a_st+a
                                        inkex.etree.SubElement(self.outputLayer, inkex.addNS('path', 'svg'),
                                             {
                                                'style': BIARC_STYLE['biarc%s' % (arcn%2)],
                                                 inkex.addNS('cx','sodipodi'):     str(c[0]),
                                                 inkex.addNS('cy','sodipodi'):     str(c[1]),
                                                 inkex.addNS('rx','sodipodi'):     str(r),
                                                 inkex.addNS('ry','sodipodi'):     str(r),
                                                 inkex.addNS('start','sodipodi'):  str(a_st),
                                                 inkex.addNS('end','sodipodi'):    str(a_end),
                                                 inkex.addNS('open','sodipodi'):  'true',
                                                 inkex.addNS('type','sodipodi'):   'arc'
                                            })
                                    # else:
                                        # inkex.errormsg("GRAH! " + str(s) + str(si))
                                s = si
                        else: # show paths prior to bi-arc approximation
                            pathString = cubicsuperpath.formatPath(operation['path'])
                            elt = inkex.etree.SubElement(self.outputLayer, inkex.addNS('path', 'svg'), \
                                {'style': 'stroke:#FF0000;fill:none;stroke-width:0.1px;stroke-linecap:round;stroke-linejoin:round', 'd': pathString})
                            if operation['id']:
                                elt.set('id', str(operation['id']) + '_gcode')
        if self.options.homeAfter:
            post.home()
        post.end()

    def _CollectItems(self, node, laserLayer, transformMatrix = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], withinSelection = False):
        # make sure we aren't recursively processing the output layer
        if node is self.outputLayer:
            return
        # make sure we aren't trying to process a hidden item
        if simplestyle.parseStyle(node.get('style')).get('display', '') == 'none':
            return
        # update the net transformation matrix if necessary
        transformString = node.get('transform', None)
        if transformString:
            transformMatrix = simpletransform.composeTransform(transformMatrix, simpletransform.parseTransform(transformString))
        # handle an element (if we care about it)
        handler = self.TAG_HANDLERS.get(node.tag)
        if handler:
            if withinSelection or ((not self.selected) or (node in self.selected.values())):
                withinSelection = True
                handler(node, transformMatrix, laserLayer)
        else:
            # for debugging, log unprocessed elements
            if node.tag in self.TAG_IGNORE_LIST or node.tag is inkex.etree.Comment:
                # don't bother processing children of elements on the ignore list
                return
            else:
                inkex.errormsg('WARNING: unhandled element in SVG document: ' + str(node.tag))
        # pass down the layer name to children we'll process later
        if node.tag == inkex.addNS('g', 'svg') and node.get(inkex.addNS('groupmode', 'inkscape')) == 'layer':
            layerName = node.get(inkex.addNS('label', 'inkscape'))
            if layerName:
                laserLayer = LaserLayer(layerName, self.defaultLayerSettings)
                self.laserLayers.append(laserLayer)
        # process any children, relevant for containers like layers/groups
        for child in node.iterchildren():
            self._CollectItems(child, laserLayer, transformMatrix, withinSelection)

    def _UseOptions(self):
        # build the output file path
        self.filePath = os.path.join(self._FixedDirectory(self.options.directory), self._FixedFilename(self.options.filename))
        # determine the scalar for mapping SVG pixels -> CNC coordinates
        if self.options.units == 'in':
            self.scaleFactor = 1.0/self.inkscapeDPI
        else: # default to mm for invalid options
            self.scaleFactor = self.MM_PER_INCH/self.inkscapeDPI
        # TODO warning messages when rejecting settings
        self.defaultLayerSettings['rapidFeedrate'] = abs(self.options.rapidFeed)
        self.defaultLayerSettings['feedrate'] = abs(self.options.feedrate)
        self.defaultLayerSettings['power'] = self.options.power/100.0
        if self.defaultLayerSettings['power'] < 0.0:
            self.defaultLayerSettings['power'] = 0.0
        if self.options.passes >= 0:
            self.defaultLayerSettings['passes'] = self.options.passes
        self.defaultDPI = abs(self.options.dpi)
        GRBLPost.SPINDLE_MIN = abs(self.options.spindleMin)
        GRBLPost.SPINDLE_MAX = abs(self.options.spindleMax)

    def _ParseLength(self, val):
        UNIT_SCALARS = { \
            'px': 1.0,
            'pt': 1.25,
            'pc': 15,
            'cm': self.inkscapeDPI/2.54,
            'mm': self.inkscapeDPI/25.4,
            'in': self.inkscapeDPI
        }
        val = val.strip()
        ending = val[-2:].lower()
        if ending in UNIT_SCALARS:
            return float(val[:-2])*UNIT_SCALARS[ending]
        return float(val)

    def effect(self):
        # get the XML DOM object
        svg = self.document.getroot()
        # determine the version of Inkscape that was used to produce the SVG file
        versionString = svg.get(inkex.addNS('version', 'inkscape'))
        if versionString:
            try:
                versionAsFloat = float('.'.join(versionString.split()[0].split('.')[0:2]))
                self.inkscapeDPI = 96.0
                #inkex.errormsg("NOTE: detected Inkscape version >= 0.92, assuming 96.0 DPI instead of 90.0")
            except:
                pass
        # load options that have by now been set by the user interface/command line, this updates self.scaleFactor using inkscapeDPI and the chosen units
        self._UseOptions()
        # delete an existing output layer so we don't keep accumulating them
        LASER_PATHS_OUTPUT_LAYER_NAME = '--Laser Paths--'
        for node in svg.iterchildren():
             if node.tag == inkex.addNS('g', 'svg') and node.get(inkex.addNS('groupmode', 'inkscape')) == 'layer' and node.get(inkex.addNS('label', 'inkscape')) == LASER_PATHS_OUTPUT_LAYER_NAME:
                svg.remove(node)
        # possibly create an output layer for debugging the generation of G-code
        if self.options.drawCurves:
            self.outputLayer = inkex.etree.SubElement(svg, inkex.addNS('g', 'svg'))
            self.outputLayer.set(inkex.addNS('groupmode', 'inkscape'), 'layer')
            self.outputLayer.set(inkex.addNS('label', 'inkscape'), LASER_PATHS_OUTPUT_LAYER_NAME)
        # TODO remember why I put this following line here :-D
        self.laserLayers.append(LaserLayer('non-layer items', self.defaultLayerSettings, True)) # default "layer" for stray items outside of all layers
        # get the SVG element dimensions
        svgEltSize = None
        try:
            svgEltSize = [self._ParseLength(svg.get('width', '0.0')), self._ParseLength(svg.get('height', '0.0'))]
        except ValueError:
            inkex.errormsg("ERROR: couldn't parse the SVG's width/height... maybe bad units? width=\"%s\", height=\"%s\"" % (svg.get('width'), svg.get('height')))
            raise
        # account for the view box as a top level transform
        viewboxTransformMatrix = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]
        svgViewBoxStr = svg.get('viewBox')
        if svgViewBoxStr:
            try:
                viewBoxNumbers = map(float, svgViewBoxStr.split())
                # TODO make sure the translate component works correctly!
                viewboxTransformMatrix = [[svgEltSize[0]/viewBoxNumbers[2], 0.0, -viewBoxNumbers[0]], [0.0, svgEltSize[1]/viewBoxNumbers[3], -viewBoxNumbers[1]]]
            except:
                inkex.errormsg("ERROR: couldn't parse the SVG's viewBox. Expecting only 'user units', maybe units were specified? viewBox=\"%s\"" % (svgViewBoxStr))
                raise
        else:
            inkex.errormsg("WARNING: no viewBox attribute on the SVG element")
        # crawl the document to gather paths/images
        #inkex.errormsg("DEBUG:" + svgViewBoxStr + " -- " + str(viewboxTransformMatrix))
        self._CollectItems(svg, self.laserLayers[-1], viewboxTransformMatrix)
        # actually emit the G-code and possibly display the resulting vectors in Inkscape
        POST_PROCESSORS = { \
            'smoothie': SmoothiewarePost,
            'grbl': GRBLPost
        }
        postFactory = POST_PROCESSORS.get(self.options.mainboard)
        if not postFactory:
            inkex.errormsg("ERROR: post-processor (G-code generator) for \"%s\" mainboard not implemented!" % (self.options.mainboard))
            return
        self._ProcessOperations(postFactory(self.options.units), svgEltSize[1])

if __name__ == "__main__":
    e = LaserCutterGcodeExportEffect()
    e.affect()
    inkex.errormsg('*** Done Processing ***')
