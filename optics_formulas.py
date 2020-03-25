from collections import namedtuple
from typing import NamedTuple
from math import sin
from math import cos
from math import log
from math import log10
from math import log2
from math import degrees
from math import radians
from math import atan
from math import tan
from math import sqrt
from math import pi
from math import e
from math import inf
from math import isfinite
from math import nan
from math import pow

class OpticsConstants:
    """Definition of Constants used for Calculations"""

    # Sensor Type Constants
    SENSOR_FF = "FF"
    SENSOR_APSC = "APSC"
    SENSOR_MFT = "MFT"
    SENSOR_MFT32 = "MFT32"
    SENSOR_1_26 = "1/2.6"
    SENSOR_1_26_43 = "1/2.6_43"
    SENSOR_1_INCH = "1Inch"
    SENSORS = [SENSOR_FF,SENSOR_APSC,SENSOR_MFT,SENSOR_MFT32,SENSOR_1_26,SENSOR_1_26_43,SENSOR_1_INCH]
            
    #Variables Used / Units in [...]
    #sensorType Denotion, as defined by Constants SENSOR_###
    #sensorDim: Property of SensorType, as defined by constants DIMENSION_###
    # f: Focal Length [mm]
    # k: Aperture Number (eg f4 > k=4) [1]
    # dist: Focussing Distance [m]
    # lambda: Wavelength [nm]
    # CoC: Circle of Confusion [micrometer]
    # m: Magnification [1]
    # megapixels: number of megapixels on sensor

    # Sensor Spec Constants
    DIMENSION = "Dimension"
    DIMENSION_WIDTH = "Width_mm"
    DIMENSION_HEIGHT = "Height_mm"
    DIMENSION_DIAGONAL = "Diagonal_mm"
    DIMENSION_CROP = "Crop_1"
    DIMENSION_RATIO = "Ratio_1"
    DIMENSION_AREA = "Area_mm2"
    DIMENSION_PIXEL_WIDTH = "PixelWidth_um"
    DIMENSION_PIXEL_HEIGHT = "PixelHeight_um"
    DIMENSION_MEGAPIXEL_NUMBER = "MegaPixelNumber_MP"
    DIMENSION_PIXEL_NUM_WIDTH = "PixelNumWidth_1"
    DIMENSION_PIXEL_NUM_HEIGHT = "PixelNumHeight_1"
    DIMENSION_PIXEL_NUM_DIAGONAL = "PixelNumDiagonal_1"
    DIMENSION_LP_PER_PICTURE_HEIGHT = "LinePairsPerPictureHeight_1PerMM" #Line Pairs per Picture Height
    DIMENSION_LP_PER_MILIMETER = "LinePairsPerMilimeter_1PerMM" #Line Pairs per Milimeter (on Sensor Plane)
    DIMENSIONS = [ 
        DIMENSION_WIDTH,DIMENSION_HEIGHT,DIMENSION_DIAGONAL,DIMENSION_CROP,DIMENSION_RATIO,
        DIMENSION_AREA,DIMENSION_PIXEL_WIDTH,DIMENSION_PIXEL_HEIGHT,DIMENSION_LP_PER_PICTURE_HEIGHT,
        DIMENSION_LP_PER_MILIMETER, DIMENSION_PIXEL_NUM_HEIGHT,DIMENSION_PIXEL_NUM_WIDTH,
        DIMENSION_MEGAPIXEL_NUMBER ]
    
    # Sensor Dimensions
    # Full Frame
    # APS C (1.5 Crop)
    # MFT (2,0 Crop)
    # MFT (MFT Cropped to 3:2)
    # LG G4 Smartphone (6,3 Crop/1(2.6'' Sensor)
    # LG G4 Smartphone (6,3 Crop/1(2.6'' Sensor with f:4.42mm )
    # LG G4 Smartphone (7,8 Crop/1(2.6'' Sensor with f:4.42mm in 4:3 Mode)
    # 1 Inch Sensor
    SENSOR_DIMENSIONS = {
        SENSOR_FF :      { DIMENSION_WIDTH:36.0,DIMENSION_HEIGHT:24.0 },
        SENSOR_APSC :    { DIMENSION_WIDTH:23.7,DIMENSION_HEIGHT:15.6 },
        SENSOR_MFT :     { DIMENSION_WIDTH:17.3,DIMENSION_HEIGHT:13.0 },
        SENSOR_MFT32 :   { DIMENSION_WIDTH:17.3,DIMENSION_HEIGHT:11.5 },
        SENSOR_1_26 :    { DIMENSION_WIDTH:06.0,DIMENSION_HEIGHT:03.4 },
        SENSOR_1_26_43 : { DIMENSION_WIDTH:04.5,DIMENSION_HEIGHT:03.4 },
        SENSOR_1_INCH :  { DIMENSION_WIDTH:13.2,DIMENSION_HEIGHT:08.8 }
    }

    # Directions for Field Of View Calculations
    DIRECTION      = "Direction"
    DIRECTION_HORI = "DirectionHorizontal"
    DIRECTION_VERT = "DirectionVertical"
    DIRECTION_DIAG = "DirectionDiagonal"  
    DIRECTIONS = [ DIRECTION_HORI,DIRECTION_VERT,DIRECTION_DIAG ]
    MAP_DIMENSION2DIRECTION = dict(zip((DIMENSION_WIDTH,DIMENSION_HEIGHT,DIMENSION_DIAGONAL),
                                       (DIRECTION_HORI,DIRECTION_VERT,DIRECTION_DIAG)))
    MAP_DIMENSION2PIXELNUM = dict(zip((DIMENSION_WIDTH,DIMENSION_HEIGHT,DIMENSION_DIAGONAL),
                                      (DIMENSION_PIXEL_NUM_WIDTH,DIMENSION_PIXEL_NUM_HEIGHT,
                                       DIMENSION_PIXEL_NUM_DIAGONAL)))
    # Standard Constants
    STANDARD_WAVELENGTH = 550
    STANDARD_ISO = 100
    STANDARDS = [ STANDARD_WAVELENGTH,STANDARD_ISO ]
    EARTH_ROTATION_SPEED = 360. / (24*60*60)  # 0,00416 degrees / second
    
    
    # Other Constants
    SENSOR = "Sensor"
    SENSOR_TARGET = "SensorTarget"
    FOCAL_LENGTH = "FocalLength_mm"
    DIOPTERS = "Diopters_1perm"
    FOCAL_LENGTH_CROPPED = "FocalLengthCropped_mm"
    FOCAL_LENGTH_FF = "FocalLengthFullFrame_mm"
    FOCAL_LENGTH_DIOPTER = "FocalLengthDiopter_mm"
    FOCAL_LENGTH_CLOSEUP = "FocalLengthCloseUp_mm"
    EQUIVALENT_FOCAL_LENGTH = "EquivalentFocalLength_mm"
    FOCAL4DISTANCE = "FocalForDistance_mm"
    CROP_FOCAL_LENGTH = "CropFocalLength_mm"    
    CROP_FOCAL_LENGTH_EQUIVALENT = "CropFocalLengthEquivalent_mm"  
    CROP_FOCAL_LENGTH_EFFECTIVE = "CropFocalLengthEffexctiveAfterCrop_mm" 
    CLOSEUP_FOCAL_LENGTH = "CloseupFocalLength_mm"    
    EXTENSION = "Extension_mm"   
    FIELD_OF_VIEW = "FieldOfView_deg"
    CIRCLE_OF_CONFUSION = "CircleOfConfusion_um"
    WAVELENGTH = "WaveLength_nm"
    APERTURE_NUMBER = "ApertureNumber_1"
    TIME = "Time_s"
    APERTURE4DOF = "ApertureForDof_1"    
    OPTIMUM_APERTURE = "OptimumAperture_1"
    OPTIMUM_APERTURE_COC = "OptimumApertureCoC_1"
    NOMINAL_APERTURE = "NominalAperture_1"
    EFFECTIVE_APERTURE = "EffectiveAperture_1"
    OPTIMUM_APERTURE_PIXEL_PITCH = "OptimumAperturePixelPitch_um" 
    EXPOSURE_TIME = "ExposureTime_s"
    EXPOSURE_APERTURE = "ExposureAperture_1"
    EXPOSURE_SENSITIVITY = "ExposureSensitivity_1"  
    HYPERFOCAL = "HyperfocalDistance_m"
    NEAR_POINT = "NearPoint_m"
    FAR_POINT = "FarPoint_m"
    DEPTH_OF_FIELD = "DepthOfField_m"
    DEPTH_OF_FIELD_MACRO = "DepthOfFieldMacro_mm"
    SOLID_ANGLE_IN_4PI = "SolidAngle_4Pi"
    ISO = "ISO"
    CROP = "Crop"
    CROP_RELATIVE = "CropRelative"
    LENGTH = "Length_mm"
    LENGTH_CROPPED = "LengthCropped_mm"
    RESOLUTION = "RESOLUTION_MP"
    EQUIVALENT_LENS_SPEC = "EquivalentLensSpec"
    DIFFRACTION_DISC_DIAMETER = "DiffractionDiscDiameter_um"
    EXPOSURE_VALUE = "ExposureValue_1"
    EXPOSURE_VALUE_ISO100 = "ExposureValue@ISO100_1"
    EXPOSURE_TIME = "ExposureTime_s"    

    START_APERTURE = "StartAperture_1"
    STOP_WIDTH = "StopWidth_1"
    NUM_STOPS = "NumStops_1"
    F_STOP_FACTOR = "FStopFactor_1"
    
    EARTH_ROTATION = "EarthRotation_DegreesPerSecond"
    LENGTH_PER_DEG = "SensorLength_MmPerDegree"
    PIXELS_PER_DEG = "Pixels_1PerDegree"
    LENGTH_PER_SEC = "SensorLength_MmPerSecond"
    
    PIXELS_PER_SEC = "Pixels_1PerSecond"
    PIXEL_NUMBER = "NumberPixels_1"

    ASTRO_SPEED = "AstroSpeed"
    ASTRO_500_RULE = "Astro_500_Rule_second"
    ASTRO_500_LENGTH = "Astro_500_Rule_length_mm"
    ASTRO_500_PIXEL = "Astro_500_Rule_pixels_1"
    ASTRO_NPF_RULE = "Astro_NPF_Rule_second"
    ASTRO_NPF_LENGTH = "Astro_NPF_Rule_length_mm"
    ASTRO_NPF_PIXEL = "Astro_NPF_Rule_pixels_1"    
        
    # optical constants 1/f = 1/b + 1/g; g/G = b/B; m = b/g   
    IMAGE_DISTANCE = "ImageDistance"  
    IMAGE_HEIGHT = "ImageHeight"
    OBJECT_DISTANCE = "ObjectDistance"
    OBJECT_HEIGHT = "ObjectHeight"
    MAGNIFICATION = "Magnification_1"
    MAGNIFICATION_LENS = "MagnificationLens_1"
    CLOSEUP_MAGNIFICATION = "CloseupMagnification_1"
    CLOSEUP_MAGNIFICATION_EXTENSION = "CloseupMagnificationExtension_1"    
    
    # lens equation 

    # Fisheye Projections
    # http://pt4pano.com/de/blog/samyang-f2812mm-fullframe
    # fisheye factor in PtGui https://www.ptgui.com/support.html#3_28
    INCIDENT_ANGLE = "IncidentAngle"
    ANGLE_FACTOR = "AngleFactor"
    IMAGE_PROJECTION = "IMageProjection_mm"
    PROJECTION_RECTILINEAR = "Rectilinear"
    PROJECTION_EQUIANGULAR = "Equiangular"
    PROJECTION_STEREOGRAPHIC = "Stereographic"
    PROJECTION_EQUIDISTANT = "Equidistant"
    PROJECTION_ORTHOGRAPHIC = "Orthographic"
    PROJECTION_EQUISOLID = "Equisolid"
    PROJECTION_FACTOR = "ProjectionFactor"
    PROJECTION_FUNCTION = "ProjectionFunction"
    PROJECTION = "Projection"
    PROJECTIONS = [ PROJECTION_RECTILINEAR,PROJECTION_STEREOGRAPHIC,
                    PROJECTION_EQUIDISTANT,PROJECTION_ORTHOGRAPHIC,PROJECTION_EQUISOLID ] 
    
    PROJECTION_SPEC = { PROJECTION_RECTILINEAR: 
                        { PROJECTION_FACTOR:1,
                          PROJECTION_FUNCTION:(lambda f,alpha,factor:(f*tan(radians(alpha))))},
                        PROJECTION_STEREOGRAPHIC: 
                        { PROJECTION_FACTOR:2,
                          PROJECTION_FUNCTION:(lambda f,alpha,factor:(f*factor*tan(radians(alpha)/factor)))},
                        PROJECTION_EQUIDISTANT: 
                        { PROJECTION_FACTOR:1,
                          PROJECTION_FUNCTION:(lambda f,alpha,factor:(f*radians(alpha)))},
                        PROJECTION_ORTHOGRAPHIC: 
                        { PROJECTION_FACTOR:1,
                          PROJECTION_FUNCTION:(lambda f,alpha,factor:(f*sin(radians(alpha))))},
                        PROJECTION_EQUISOLID: 
                        { PROJECTION_FACTOR:2,
                          PROJECTION_FUNCTION:(lambda f,alpha,factor:(f*factor*sin(radians(alpha)/factor)))},
                      }    

    # Fisheye Lens Projections
    # http://pt4pano.com/de/blog/samyang-f2812mm-fullframe
    # Samyangs 8mm -> R = 2,7 * f * tan ( alpha / 2,7)
    # Samyangs 7.5mm -> R = 3 * 7.3 * sin ( alpha / 3)
    # Sigma f3.5 / 8mm -> R = f alpha
    # Canon 15mm -> R = 2 * f * sin( alpha / 2 )
    # Nikkor F2.8 10,5mm -> R = 1,55*k*f*sin(alpha / 1,55)
    # Madoka F4/7.3mm -> R = f * sin ( alpha )
    # https://www.pt4pano.com/blog/2017/neue-fisheyes-fuer-panoramafotografie
    # Meike F2 / 6.5mm -> R = 1,4 * f * sin(alpha / 1,4 )
    LENS = "Lens"
    LENS_SAMYANG8 = "Samyang8F28"
    LENS_SAMYANG75 = "Samyang75F35"
    """
    https://groups.google.com/forum/#!topic/ptgui/AwTE531o7xA    
    https://www.pt4pano.com/blog/2017/neue-fisheyes-fuer-panoramafotografie
    When bringing them into PTGui you should get the Focal length dialog.
    Enter the focal length (6.5) then you should get the Meike in the Lens
    type droplist
    To alter this later go to Project Assistant tab, under "Set up panorama"
    click the link after "Lens:" to get the Focal length dialog. Proceed as
    above.
    """
    LENS_MEIKE65   = "Meike65F2"
    LENS_SIGMA8F35 = "Sigma8F35"
    LENS_CANON15 = "Canon15"
    LENS_NIKON10 = "Nikon10"
    LENS_MADOKA = "Madoka"  
    LENSES = [ 
        LENS_SAMYANG8, LENS_SAMYANG75, LENS_MEIKE65,LENS_SIGMA8F35,
        LENS_CANON15,LENS_NIKON10,LENS_MADOKA ]

    # Fisheye Lens Projections
    # http://pt4pano.com/de/blog/samyang-f2812mm-fullframe
    # Samyangs 8mm -> R = 2,7 * f * tan ( alpha / 2,7)
    # Samyangs 7.5mm -> R = 2.7 * 7.3 * sin ( alpha / 2.7)
    # Sigma f3.5 / 8mm -> R = f alpha
    # Canon 15mm -> R = 2 * f * sin( alpha / 2 )
    # Nikkor F2.8 10,5mm -> R = 1,55*k*f*sin(alpha / 1,55)
    # Madoka F4/7.3mm -> R = f * sin ( alpha )
    # https://www.pt4pano.com/blog/2017/neue-fisheyes-fuer-panoramafotografie
    # Meike F2 / 6.5mm -> R = 1,4 * f * sin(alpha / 1,4 )    
    FISHEYE_LENS_SPECS = {LENS_SAMYANG8:{ FOCAL_LENGTH:8.,
                                          PROJECTION_FUNCTION:PROJECTION_STEREOGRAPHIC,
                                          PROJECTION_FACTOR:2.700},
                          LENS_SAMYANG75:{FOCAL_LENGTH:7.5,
                                          PROJECTION_FUNCTION:PROJECTION_EQUISOLID,
                                          PROJECTION_FACTOR:2.700},
                          LENS_MEIKE65:{  FOCAL_LENGTH:6.5,
                                          PROJECTION_FUNCTION:PROJECTION_EQUISOLID,
                                          PROJECTION_FACTOR:1.400},
                          LENS_SIGMA8F35:{FOCAL_LENGTH:8.0,
                                          PROJECTION_FUNCTION:PROJECTION_EQUIDISTANT,
                                          PROJECTION_FACTOR:None},
                          LENS_CANON15:{  FOCAL_LENGTH:15.0,
                                          PROJECTION_FUNCTION:PROJECTION_EQUISOLID,
                                          PROJECTION_FACTOR:2.000},
                          LENS_NIKON10:{  FOCAL_LENGTH:10.5,
                                          PROJECTION_FUNCTION:PROJECTION_EQUISOLID,
                                          PROJECTION_FACTOR:1.500},
                          LENS_MADOKA:{   FOCAL_LENGTH:7.3,
                                          PROJECTION_FUNCTION:PROJECTION_ORTHOGRAPHIC,
                                          PROJECTION_FACTOR:None}  } 
    
class OpticsCalculator:
    """" Performs Opticál Calculations Useful for Photography """

    @staticmethod
    def unfreeze(v):
        """ Unfreeze a single calculation result: reverses frozenset key back to dict 
            and extracts result dictionary as key value tuple
        """
        return (dict(list(v.keys())[0]),list(v.values())[0])
    
    @staticmethod
    def convert_to_tuple(name,d):
        """ turns dictionary into named & hashable tuple. keys are sorted 
            can be reversed by calling _asdict() """
        result = None

        if isinstance(d,dict):
            NamedTuple = namedtuple(name, sorted(d))
            result = NamedTuple(**d)
        return result
    
    @staticmethod
    def bootstrap(sensor_type=OpticsConstants.SENSOR_FF,megapixels=None):
        """get variables and sensor specs"""
        return (OpticsConstants,OpticsCalculator,
                OpticsCalculator.get_sensor_specs(sensor_type=sensor_type,megapixels=megapixels,with_keys=False))
    
    @staticmethod
    def get_results(result_dict,with_keys=False,tuple_name="",key_dict=None):
        """ Helper method to get either results as dictionary 
            or additionally with supplied input fields """
        o = OpticsCalculator
        
        if with_keys is True:
            key = o.convert_to_tuple(tuple_name,key_dict)
            result = {key:result_dict}
        else:
            result = result_dict
        return result
    
    @staticmethod
    def get_sensor_specs(sensor_type,megapixels=None,with_keys=False):
        """ #01 Calculates Sensor Specs 
        If Pixel Nuber (in Megapixels) is given then additional specs are calculated
        If with_keys is set it will return a dictionary with input values as named tuple,
        otherwise only result values
        """
        c = OpticsConstants
        o = OpticsCalculator
        if sensor_type not in c.SENSORS:
            print(f"Can't find Sensor Spec {sensor_type}, allowed: {c.SENSORS}")
            return None
        sensorDim = c.SENSOR_DIMENSIONS[sensor_type]
        sensorSpecs = sensorDim.copy()
        w = sensorSpecs[c.DIMENSION_WIDTH]
        h = sensorSpecs[c.DIMENSION_HEIGHT]
        d = sqrt( w**2 + h**2 )
        sensorSpecs[c.DIMENSION_DIAGONAL] = d
        sensorSpecs[c.CIRCLE_OF_CONFUSION] = 1000 * ( d / 1500. )
        d_crop = sqrt( 24**2 + 36**2 )
        sensorSpecs[c.DIMENSION_CROP] = d_crop / float( sensorSpecs[c.DIMENSION_DIAGONAL] )
        sensorSpecs[c.DIMENSION_RATIO] = w / float(h)
        sensorSpecs[c.DIMENSION_AREA] = w * float(h)
        if megapixels is not None:
            sensorSpecs[c.DIMENSION_PIXEL_NUM_WIDTH]  = int(1000*sqrt(megapixels*sensorSpecs[c.DIMENSION_RATIO]))
            sensorSpecs[c.DIMENSION_PIXEL_NUM_HEIGHT] = int( sensorSpecs[c.DIMENSION_PIXEL_NUM_WIDTH]
                                                        / sensorSpecs[c.DIMENSION_RATIO])
            sensorSpecs[c.DIMENSION_PIXEL_NUM_DIAGONAL] =  int ( sqrt ( sensorSpecs[c.DIMENSION_PIXEL_NUM_WIDTH]**2 + \
                                                                  sensorSpecs[c.DIMENSION_PIXEL_NUM_HEIGHT]**2 ) )   
            # Pixel Pitch in nm
            sensorSpecs[c.DIMENSION_PIXEL_WIDTH] = 1000 * w / sensorSpecs[c.DIMENSION_PIXEL_NUM_WIDTH] 
            sensorSpecs[c.DIMENSION_PIXEL_HEIGHT] = 1000 * h / sensorSpecs[c.DIMENSION_PIXEL_NUM_HEIGHT]
            ' Line Pairs per Milimeter'
            sensorSpecs[c.DIMENSION_LP_PER_MILIMETER] = sensorSpecs[c.DIMENSION_PIXEL_NUM_HEIGHT] / (2.0 * h)
            sensorSpecs[c.DIMENSION_LP_PER_PICTURE_HEIGHT] = sensorSpecs[c.DIMENSION_PIXEL_NUM_HEIGHT] / 2.0
        key_dict = {c.SENSOR:sensor_type,c.DIMENSION_MEGAPIXEL_NUMBER:megapixels}
        result = o.get_results(result_dict=sensorSpecs,with_keys=with_keys,
                               tuple_name=c.SENSOR,key_dict=key_dict)
        result = {k: round(v,2) for k, v in result.items()}
        
        return result
            
    @staticmethod
    def get_field_of_view(focal_length,sensor_type=OpticsConstants.SENSOR_FF,with_keys=False):
        """ #02 Calculate Field Of View Angles for given sensor and focal length 
            Field Of View is also calculated as fraction of unit sphere 4Pi     
            Field of view as fraction of a sphere / 4 Pi 
            (4 Pi Sterads corresponds to the surface angle on a sphere of radius 1)
            One infinitesimal area on the sphere is ( r sin v dv ) * ( r dh ) 
            (h:horizontal angle v:vertical angle)
            For finite angles v and h: INT(r*r*sin v dv dh)|(0...h)(0...v) 
            h:v:horizontal vertical field of view (#4a)
            Area Angle OMEGA = INT(r*r*sin v dv dh)|(0...h)(0...v) / r^2
                    => OMEGA = h * (-cos v  + cos 0  ) = h * ( 1 - cos v )
            Whole spehere: h = 2 Pi / v = Pi > OMEGA(Sphere) = 2 Pi (1 - cos Pi ) = 4 Pi
            OmegaIn4Pi = OMEGA / 4 Pi            
        """
        c,o,specs = OpticsCalculator.bootstrap(sensor_type)
        specs_dim = [ specs[c.DIMENSION_WIDTH],specs[c.DIMENSION_HEIGHT],specs[c.DIMENSION_DIAGONAL]]
        
        fov = list(map(lambda d:degrees(2*atan(d/(2*focal_length))),specs_dim))
        
        fov_keys = [ c.DIRECTION_HORI,c.DIRECTION_VERT,c.DIRECTION_DIAG ]
        result_dict = dict(zip(fov_keys, fov))     
        # calculate fraction of sphere
        h = radians(result_dict[c.DIRECTION_HORI])
        v = radians(result_dict[c.DIRECTION_VERT])
        omega_4pi = ( h * (1 - cos(v))) / (4*pi)
        result_dict[c.SOLID_ANGLE_IN_4PI] = omega_4pi
        key_dict = {c.SENSOR:sensor_type,c.FOCAL_LENGTH:focal_length}
        result = o.get_results(result_dict=result_dict,with_keys=with_keys,
                               tuple_name=c.FIELD_OF_VIEW,key_dict=key_dict)  
        return result        
    
    @staticmethod
    def get_focal4distance(obj_dist,obj_height,sensor_type=OpticsConstants.SENSOR_FF,
                           dimension=OpticsConstants.DIMENSION_WIDTH,with_keys=False):
        """ #03 Based on lens equation, calculates focal length and fov in mm 
            for given object distance obj_dist [m] and object size obj_size [m]
            (answers how much focal length is required to cover the whole sensor)
            https://de.wikipedia.org/wiki/Linsengleichung
            f = (im_height * obj_dist) / (im_height + obj_height)  
        """
        c,o,specs = OpticsCalculator.bootstrap(sensor_type)
        
        im_height = specs[dimension] / 1000.
        
        direction = c.MAP_DIMENSION2DIRECTION[dimension]
        '''
        direction = c.DIRECTION_HORI        
        if dimension == c.DIMENSION_DIAGONAL:
            direction = c.DIRECTION_HORI
        elif dimension == c.DIMENSION_HEIGHT:
            direction = c.DIRECTION_VERT
        '''
        key_list = (c.OBJECT_DISTANCE,c.OBJECT_HEIGHT,c.IMAGE_HEIGHT,c.SENSOR,c.DIRECTION)
        key_values = (obj_dist,obj_height,im_height,sensor_type,direction)
        key_dict = dict(zip(key_list,key_values))

        f = round( 1000 * (im_height*obj_dist) / (im_height+obj_height),2)
        fov = o.get_field_of_view(f,sensor_type=sensor_type,with_keys=False)
        result_dict = { c.FOCAL_LENGTH:f,c.FIELD_OF_VIEW:fov[direction] }  
        result = o.get_results(result_dict=result_dict,with_keys=with_keys,
                               tuple_name=c.FOCAL4DISTANCE,key_dict=key_dict)   
        return result      
    
    @staticmethod
    def get_dof(f,k,sensor_type=OpticsConstants.SENSOR_FF,distance=None,with_keys=False):
        """ #4a Returns hyperfocal distance, near point, far point, depth of field
            Hyperfocal Distance in [m]
            ( https://de.wikipedia.org/wiki/Hyperfokale_Entfernung )
            f: Focal Length (mm)
            k: Aperture Number, eg Aperture F/4 -> k=4 (1)
            {sensorType:FF,APSC,MFT}, determines Circle Of Confusion
            hyperfocal = f*f / k*Z + f
            Depth Of Field Calculation / Near Point
            https://de.wikipedia.org/wiki/Sch%C3%A4rfentiefe
            https://en.wikipedia.org/wiki/Depth_of_field
            dn: Near Limit (m) / df: far Limit (m) / 
            dist: Subject Distance in m
            DOF_Near = distance * (hyperfocal - F) / ((hyperfocal - F) + (distance - F))
            DOF_Far  = distance * (hyperfocal - F) / ((hyperfocal - F) + (F - distance))|for distance < hyperfocal         
        """
        c,o,specs = OpticsCalculator.bootstrap(sensor_type)
        
        # circle of confusion in m
        Z = specs[c.CIRCLE_OF_CONFUSION] / 1000**2.
        # focal length given in mm in meters"
        F = f / 1000
        hyperfocal = round(( F + ( F**2 / ( k * Z ) ) ),3)
        if distance is not None:
            dof_near = round(distance * (hyperfocal - F) / ( (hyperfocal - F) + (distance - F) ),3)
            if ( distance < hyperfocal ):
                dof_far =  round(distance * (hyperfocal - F) / ( (hyperfocal - F) + (F - distance) ),3)
                dof = round(dof_far - dof_near,3)
            else:
                dof_far = inf
                dof = inf
        else:
            dof_near = nan
            dof_far  = nan
            dof = nan
        
        key_dict = {c.FOCAL_LENGTH:f,c.APERTURE_NUMBER:k,c.SENSOR:sensor_type,
                    c.OBJECT_DISTANCE:distance}
        
        result_dict = { c.CIRCLE_OF_CONFUSION:round(specs[c.CIRCLE_OF_CONFUSION],3),
                        c.NEAR_POINT:dof_near,c.FAR_POINT:dof_far,
                        c.HYPERFOCAL:hyperfocal, c.DEPTH_OF_FIELD:dof }
        result = o.get_results(result_dict=result_dict,with_keys=with_keys,
                               tuple_name=c.DEPTH_OF_FIELD,key_dict=key_dict)           
        return result

    @staticmethod
    def get_aperture_for_dof(f,distance,dof,sensor_type=OpticsConstants.SENSOR_FF,with_keys=False):
        """ #4b Calculate Aperture on given Depth Of Field
            http://www.elmar-baumann.de/fotografie/schaerfentiefe/node25.html
            k = F*F*(sqrt(dist*dist+DOF*DOF)-dist)/((DOF*CoC)*(dist-F))
            DOF: Depth Of Field/dist:Focus Distance/f:Focal Length/CoC:Circle Of Confusion               
        """
        c,o,specs = OpticsCalculator.bootstrap(sensor_type)
        
        # circle of confusion in um
        z = specs[c.CIRCLE_OF_CONFUSION]
        Z = z / 1000000.
        # focal length given in mm in meters"
        F = f / 1000
        aperture_for_dof = (F**2) * ( ( sqrt(distance**2+dof**2)-distance ) / 
                                    ( ( dof * Z ) * ( distance - F ) ) ) 
        params_in = dict( zip( (c.FOCAL_LENGTH,c.OBJECT_DISTANCE,c.DEPTH_OF_FIELD,c.SENSOR),
                            (f,distance,dof,sensor_type) ) )        
        params_out = dict( zip( (c.CIRCLE_OF_CONFUSION,c.APERTURE_NUMBER),
                                (z,aperture_for_dof) ) )
        result = o.get_results(result_dict=params_out,with_keys=with_keys,
                               tuple_name=c.APERTURE4DOF,key_dict=params_in)
        return result
    
    @staticmethod
    def get_equivalent_focal_length(fov,sensor_type=OpticsConstants.SENSOR_FF,
                                    direction=OpticsConstants.DIRECTION_HORI,with_keys=False):
        """ #5 Equivalent Focal Length for given Field Of View given in degrees                     
        """
        c,o,_ = OpticsCalculator.bootstrap(sensor_type=sensor_type)
        if direction == c.DIRECTION_HORI:
            sensor_spec = c.DIMENSION_WIDTH
        elif direction == c.DIRECTION_VERT:
            sensor_spec = c.DIMENSION_HEIGHT
        else:
            sensor_spec = c.DIMENSION_DIAGONAL
        sensor_length = o.get_sensor_specs(sensor_type=sensor_type,with_keys=False)[sensor_spec] / 2.
        angle_rad = radians(fov/2)
        equivalent_focal_length = round((sensor_length / tan(angle_rad)),0)
        params_in  = dict( zip( (c.FIELD_OF_VIEW,c.SENSOR,c.DIRECTION),
                                (fov,sensor_type,direction) ) )        
        params_out = dict( zip( (c.DIMENSION,c.LENGTH,c.FOCAL_LENGTH),
                                (sensor_spec,sensor_length,equivalent_focal_length) ) )
        result = o.get_results(result_dict=params_out,with_keys=with_keys,
                               tuple_name=c.EQUIVALENT_FOCAL_LENGTH,key_dict=params_in)
        return result
    
    @staticmethod
    def get_crop_focal_length_equivalent(f,crop,sensor_type=OpticsConstants.SENSOR_FF,
                                         direction=OpticsConstants.DIRECTION_HORI,with_keys=False):
        """ #6 Equivalent Focal Length after Crop was applied  
            CropFactor: Value between 0 and 1 (1:No Crop 0:Cropped into nothing)
        """    
        
        # get sensor Specs and apply crop factor
        c,o,_ = OpticsCalculator.bootstrap(sensor_type=sensor_type)
        if direction == c.DIRECTION_HORI:
            sensor_spec = c.DIMENSION_WIDTH
        elif direction == c.DIRECTION_VERT:
            sensor_spec = c.DIMENSION_HEIGHT
        else:
            sensor_spec = c.DIMENSION_DIAGONAL
            
        sensor_length_cropped = o.get_sensor_specs(sensor_type=sensor_type,with_keys=False)[sensor_spec] * crop
        # get same spec for full frame sensor
        # specs_ff = o.get_sensor_specs(sensor_type=c.SENSOR_FF,with_keys=False)
        sensor_length_cropped_ff = o.get_sensor_specs(sensor_type=c.SENSOR_FF,with_keys=False)[sensor_spec] * crop
        crop_ff = sensor_length_cropped_ff / sensor_length_cropped
        # get new cropped field of view
        fov_cropped = degrees ( 2 * atan(sensor_length_cropped / (2 * f)) )
        cropped_focal_length = o.get_equivalent_focal_length(fov_cropped,sensor_type=sensor_type,
                                    direction=direction,with_keys=False)
        cropped_focal_length_ff = round( cropped_focal_length * crop_ff,3)
        params_in  = dict( zip( (c.FOCAL_LENGTH,c.CROP,c.SENSOR,c.DIRECTION),
                                (f,crop,sensor_type,direction) ) )        
        params_out = dict( zip( (c.DIMENSION,c.LENGTH_CROPPED,c.CROP,c.FIELD_OF_VIEW,c.CROP_FOCAL_LENGTH_EQUIVALENT,c.FOCAL_LENGTH_FF),
                                (sensor_spec,sensor_length_cropped,crop_ff,fov_cropped,cropped_focal_length,cropped_focal_length_ff) ) )      
        result = o.get_results(result_dict=params_out,with_keys=with_keys,
                               tuple_name=c.CROP_FOCAL_LENGTH_EQUIVALENT,key_dict=params_in)
        return result
    
    @staticmethod
    def get_diffraction_dmeter(k,lambda_nm=OpticsConstants.STANDARD_WAVELENGTH,with_keys=False):
        """ #7 gets the diffraction disc diameter in um for given aperture number and wavelength lambda
           default wavelength is 550nm 
           Diffraction Disc Radius
           https://en.wikipedia.org/wiki/Diffraction-limited_system#Implications_for_digital_photography
           https://de.wikipedia.org/wiki/Kritische_Blende
           https://de.wikipedia.org/wiki/Beugungsunsch%C3%A4rfe
           https://de.wikipedia.org/wiki/Numerische_Apertur
           Returns Size of Diffraction Disc in Micrometers for a wavelength lambda (in nm)
        """ 
        c,o,_ = OpticsCalculator.bootstrap()
        diffraction_disc_diameter = ( 1.22 * lambda_nm * k ) / 1000

        params_in  = dict( zip( (c.APERTURE_NUMBER,c.WAVELENGTH),
                                (k,lambda_nm) ) )        
        params_out = dict( zip( (c.DIFFRACTION_DISC_DIAMETER),
                                (diffraction_disc_diameter) ) )        
        result = o.get_results(result_dict=params_out,with_keys=with_keys,
                               tuple_name=c.DIFFRACTION_DISC_DIAMETER,key_dict=params_in)
        return result        
    
    @staticmethod
    def get_optimum_aperture(sensor_type=OpticsConstants.SENSOR_FF,
                             lambda_nm=OpticsConstants.STANDARD_WAVELENGTH,
                             magnification=0,coc=None,with_keys=False):
        """ #8 Returns Optimum Aperture ( CoC in the range of Diffraction)
            http://www.elmar-baumann.de/fotografie/rechner/rechner-foerderliche-blende.html
            https://www.elmar-baumann.de/fotografie/lexikon/blende-effektive.html
            https://www.elmar-baumann.de/fotografie/lexikon/blende-foerderliche.html
            https://de.wikipedia.org/wiki/Kritische_Blende
            https://de.wikipedia.org/wiki/Beugungsscheibchen
            http://foto-net.de/net/objektive/licht.html
            CoC:Circle Of Confusion in MicroMeters
            lambda: Wavelength in nm
            m:Magnification
        """
        # get circle of confusion in um
        c,o,specs = OpticsCalculator.bootstrap(sensor_type=sensor_type)
        if coc is None:
            coc = specs[c.CIRCLE_OF_CONFUSION]
        else:
            coc = coc
        optimum_aperture_effective = 1000 * coc / ( 1.22 * lambda_nm )
        optimum_aperture_nominal = optimum_aperture_effective / (magnification+1)

        params_in  = dict( zip( (c.SENSOR,c.WAVELENGTH,c.MAGNIFICATION),
                                (sensor_type,lambda_nm,magnification) ) )        
        params_out = dict( zip( (c.CIRCLE_OF_CONFUSION,c.EFFECTIVE_APERTURE,c.NOMINAL_APERTURE),
                                (coc,optimum_aperture_effective,optimum_aperture_nominal) ) )   
        result = o.get_results(result_dict=params_out,with_keys=with_keys,
                               tuple_name=c.OPTIMUM_APERTURE,key_dict=params_in)
        return result

    @staticmethod
    def get_magnification(distance,f,with_keys=False):    
        """ #9 Returns Magnification Factor (based on lens equation)
            https://de.wikipedia.org/wiki/Linsengleichung
            distance:Object Distance in m, f:focal Length in mm
        """
        c,o,_ = OpticsCalculator.bootstrap()
        magnification = 1 / ( ( ( distance * 1000 ) / f ) - 1 )

        params_in  = dict( zip( (c.OBJECT_DISTANCE,c.FOCAL_LENGTH),
                                (distance,f) ) )        
        params_out = dict( zip( (c.MAGNIFICATION),
                                (magnification) ) )
        result = o.get_results(result_dict=params_out,with_keys=with_keys,
                               tuple_name=c.MAGNIFICATION,key_dict=params_in)
        return result
    
    @staticmethod
    def get_optimum_aperture_pixel_pitch(sensor_type=OpticsConstants.SENSOR_FF,
                                         megapixels=24,lambda_nm=OpticsConstants.STANDARD_WAVELENGTH,
                                         magnification=0,with_keys=False):
        """ #10 Aperture calculated for Diffraction having the size of a single Pixe
            How small can the aperture be, so that the Circle of Confusion will fit into a single pixel
        """
        c,o,specs = OpticsCalculator.bootstrap(sensor_type=sensor_type,megapixels=megapixels)
        coc = specs[c.CIRCLE_OF_CONFUSION]
        pixel_width = specs[c.DIMENSION_PIXEL_WIDTH]
        opt_aperture_coc = o.get_optimum_aperture(sensor_type=sensor_type,
                                                  lambda_nm=lambda_nm,magnification=magnification,
                                                  coc=coc)
        opt_aperture_pixelpitch = o.get_optimum_aperture(sensor_type=sensor_type,
                                                         lambda_nm=lambda_nm,magnification=magnification,
                                                         coc=pixel_width)
        params_in  = dict( zip( (c.SENSOR,c.PIXEL_NUMBER,c.WAVELENGTH,c.MAGNIFICATION),
                                (sensor_type,megapixels,lambda_nm,magnification) ) )        
        params_out = dict( zip( (c.CIRCLE_OF_CONFUSION,c.DIMENSION_PIXEL_WIDTH,c.OPTIMUM_APERTURE_COC,c.OPTIMUM_APERTURE_PIXEL_PITCH),
                                (coc,pixel_width,opt_aperture_coc,opt_aperture_pixelpitch) ) )
      
        result = o.get_results(result_dict=params_out,with_keys=with_keys,
                               tuple_name=c.OPTIMUM_APERTURE_PIXEL_PITCH,key_dict=params_in)
        return result
    
    @staticmethod
    def get_exposure_value(t=1.,k=1.,iso=100.,with_keys=False):    
        """ #11a calculate exposure value
            https://de.wikipedia.org/wiki/Lichtwert
            https://en.wikipedia.org/wiki/Exposure_value
            https://www.scantips.com/lights/evchart.html
            t: Exposure Time [s] / k:Aperture Number [1] / ISO: ISO Value (sensitivity) [1]
            Exposure_AV: Aperture Value        
        """
        c,o,_ = OpticsCalculator.bootstrap()
        ev_av = log2(k**2)
        ev_tv = log2(1/t)
        ev_sv = log2(iso/100)
        #Light Value ev = ev_av + ev_tv + ev_sv = EV100 + log2(ISO / 100)   
        ev_100 = ev_av + ev_tv
        ev = ev_100 + ev_sv
        
        params_in  = dict( zip( (c.TIME,c.APERTURE_NUMBER,c.ISO),
                                (t,k,iso) ) )        
        params_out = dict( zip( (c.EXPOSURE_APERTURE,c.EXPOSURE_TIME,c.EXPOSURE_SENSITIVITY,c.EXPOSURE_VALUE,c.EXPOSURE_VALUE_ISO100),
                                (ev_av,ev_tv,ev_sv,ev,ev_100) ) )
        result = o.get_results(result_dict=params_out,with_keys=with_keys,
                               tuple_name=c.EXPOSURE_VALUE,key_dict=params_in)
        return result
    
    @staticmethod
    def get_exposure_time(ev=10.,k=1.,iso=100.,with_keys=False):    
        """ #11b calculate exposure time for given exposure value and aperture and iso  
        """
        c,o,_ = OpticsCalculator.bootstrap()
        ev_av = log2(k**2)
        ev_sv = log2(iso/100)
        ev_tv = ev - ev_av - ev_sv
        t = 1 / pow(2,ev_tv)
        params_in  = dict( zip( (c.EXPOSURE_VALUE,c.APERTURE_NUMBER,c.ISO),
                                (ev,k,iso) ) )        
        params_out = dict( zip( (c.EXPOSURE_APERTURE,c.EXPOSURE_SENSITIVITY,c.EXPOSURE_TIME,c.TIME),
                                (ev_av,ev_sv,ev_tv,t) ) )

        result = o.get_results(result_dict=params_out,with_keys=with_keys,
                               tuple_name=c.EXPOSURE_TIME,key_dict=params_in)
        return result
    
    @staticmethod
    def get_exposure_aperture(ev=10.,t=1.,iso=100.,with_keys=False):    
        """ #11c calculate exposure aperture for given exposure value and exposure time and iso  
        """   
        c,o,_ = OpticsCalculator.bootstrap() 
        ev_tv = log2(1/t)
        ev_sv = log2(iso/100)
        ev_av = ev - ev_tv - ev_sv
        k = sqrt(pow(2,ev_av))
        params_in  = dict( zip( (c.EXPOSURE_VALUE,c.TIME,c.ISO),
                                (ev,t,iso) ) )        
        params_out = dict( zip( (c.EXPOSURE_TIME,c.EXPOSURE_SENSITIVITY,c.EXPOSURE_APERTURE,c.APERTURE_NUMBER),
                                (ev_tv,ev_sv,ev_av,k) ) )
       
        result = o.get_results(result_dict=params_out,with_keys=with_keys,
                               tuple_name=c.EXPOSURE_APERTURE,key_dict=params_in)
        return result

    @staticmethod
    def get_exposure_sensitivity(ev=10.,k=16.,t=1.,with_keys=False):    
        """ #11d calculate exposure aperture for given exposure value and exposure time and aperture 
        """
        c,o,_ = OpticsCalculator.bootstrap()
        ev_tv = log2(1/t)
        ev_av = log2(k**2)
        ev_sv = ev - ev_av - ev_tv
        iso = 100 * pow(2,ev_sv)
        params_in  = dict( zip( (c.EXPOSURE_VALUE,c.APERTURE_NUMBER,c.TIME),
                                (ev,k,t) ) )        
        params_out = dict( zip( (c.EXPOSURE_TIME,c.EXPOSURE_APERTURE,c.EXPOSURE_SENSITIVITY,c.ISO),
                                (ev_tv,ev_av,ev_sv,iso) ) )
     
        result = o.get_results(result_dict=params_out,with_keys=with_keys,
                               tuple_name=c.EXPOSURE_SENSITIVITY,key_dict=params_in)
        return result
    
    @staticmethod
    def get_aperture_number(start_aperture=2.8,stop_width=4,num_stops=2,with_keys=False):
        """ #12 Calculate Aperture Number for a given number of f Stop Fractions 
            and a starting Aperture Number
            startAperture: Start Aperture
            stopWidth: number of stops for single f Stop, eg use stopWidth = 3 for 1/3 of a stop
            numStops: Number of stopWidths
            Examples: fStop(1,1,1)=1,4; fStop(1,1,2)=2; fStop(1.4,1,1)=2; fStop(1,3,3)=1.4 , ...
        """
        c,o,_ = OpticsCalculator.bootstrap()
        f_stop_factor = sqrt(pow(10,(0.3/stop_width)))
        aperture_number = round(start_aperture * pow(f_stop_factor,num_stops),1)
        params_in  = dict( zip( (c.START_APERTURE,c.STOP_WIDTH,c.NUM_STOPS),
                                (start_aperture,stop_width,num_stops) ) )        
        params_out = dict( zip( (c.F_STOP_FACTOR,c.APERTURE_NUMBER),
                                (f_stop_factor,aperture_number) ) )     
        
        result = o.get_results(result_dict=params_out,with_keys=with_keys,
                               tuple_name=c.APERTURE_NUMBER,key_dict=params_in)
        return result
    
    @staticmethod
    def get_closeup_focal_length(f,D,with_keys=False):
        ''' #13 Close Up Lens focal length (D = 1 / f ); D [1/m]; f [mm]
            D: Diopters closeup lens, f focal length
            http://www.elmar-baumann.de/fotografie/herleitungen/herleitungen-abbildungsmasstab.html
        '''
        c,o,_ = OpticsCalculator.bootstrap()
        f_D = 1000. / D
        f_closeup = 1 / ((1/f)+(1/f_D))
        
        params_in  = dict( zip( (c.FOCAL_LENGTH,c.DIOPTERS),
                                (f,D) ) ) 
                                       
        params_out = dict( zip( (c.FOCAL_LENGTH_DIOPTER,c.FOCAL_LENGTH_CLOSEUP),
                                (f_D,f_closeup) ) )  
        result = o.get_results(result_dict=params_out,with_keys=with_keys,
                               tuple_name=c.CLOSEUP_FOCAL_LENGTH,key_dict=params_in)
        return result
    
    @staticmethod
    def get_closeup_magnification_at_distance(f,D,distance=inf,with_keys=False):
        ''' #14 Close Up Lens Magnification At Focussing Distance dist (in meters)
            D: Diopters closeup lens, f focal length
            http://www.elmar-baumann.de/fotografie/herleitungen/herleitungen-abbildungsmasstab.html
        '''   
        c,o,_ = OpticsCalculator.bootstrap()     
        f_D = 1000 / D
        closeup_magnification_at_infinity = f / f_D
        if isfinite(distance): 
            distance_mm = distance * 1000.
            closeup_magnification_at_distance = ((f*(distance_mm+f_D)) / (f_D*(distance_mm-f)))
        else:
            distance_mm = inf
            closeup_magnification_at_distance = closeup_magnification_at_infinity            
        params_in  = dict( zip( (c.FOCAL_LENGTH,c.DIOPTERS,c.OBJECT_DISTANCE),
                                (f,D,distance) ) )        
        params_out = dict( zip( (c.FOCAL_LENGTH_DIOPTER,c.CLOSEUP_MAGNIFICATION),
                                (f_D,closeup_magnification_at_distance) ) )
       
        result = o.get_results(result_dict=params_out,with_keys=with_keys,
                               tuple_name=c.CLOSEUP_MAGNIFICATION,key_dict=params_in)
        return result
    
    @staticmethod
    def get_extension_closeup_magnification(f,extension=0.,D=0,magnificaton_lens=0.,with_keys=False):
        ''' #15 Magnification for Tube Extension and Close Up Lens Combined
            will simplify to extension magnification for tube extension if D is 0
            http://www.herbig-3d.de/german/kameraoptik.htm
        '''
        c,o,_ = OpticsCalculator.bootstrap()
        f_D = 1000 / D
        extension_closeup_magnification = magnificaton_lens + \
                                          ( ( f / f_D ) * (1 + magnificaton_lens) ) + \
                                          (  extension * ( (1/f) + (1/f_D)) )
        params_in  = dict( zip( (c.FOCAL_LENGTH,c.EXTENSION,c.DIOPTERS,c.MAGNIFICATION_LENS),
                                (f,extension,D,magnificaton_lens) ) )        
        params_out = dict( zip( (c.FOCAL_LENGTH_DIOPTER,c.CLOSEUP_MAGNIFICATION_EXTENSION),
                                (f_D,extension_closeup_magnification) ) )
        
        result = o.get_results(result_dict=params_out,with_keys=with_keys,
                               tuple_name=c.CLOSEUP_MAGNIFICATION_EXTENSION,key_dict=params_in)
        return result

    @staticmethod
    def get_dof_macro(k,magnification,sensor_type=OpticsConstants.SENSOR_FF,with_keys=False):
        ''' #16 DOF Calculation for macro case (neglecting Pupil Magnification)
            On Macro Photography, Check out
            http://www.cambridgeincolour.com/tutorials/macro-photography-intro.htm
            http://www.dofmaster.com/equations.html
            Formula taken from
            https://en.wikipedia.org/wiki/Depth_of_field#Close-up
        '''
        c,o,specs = OpticsCalculator.bootstrap(sensor_type=sensor_type)
        # circle of confusion in mm
        coc = specs[c.CIRCLE_OF_CONFUSION] / 1000.
        #effective aperture
        k_eff = k * ( magnification + 1 )
        dof_macro = ( 2 * k_eff * coc ) / ( magnification**2 )
        params_in  = dict( zip( (c.APERTURE_NUMBER,c.MAGNIFICATION_LENS,c.SENSOR),
                                (k,magnification,sensor_type) ) )        
        params_out = dict( zip( (c.CIRCLE_OF_CONFUSION,c.EFFECTIVE_APERTURE,c.DEPTH_OF_FIELD_MACRO),
                                (coc,k_eff,dof_macro) ) )
      
        result = o.get_results(result_dict=params_out,with_keys=with_keys,
                               tuple_name=c.DEPTH_OF_FIELD_MACRO,key_dict=params_in)
        return result

    @staticmethod
    def get_fisheye_projection(f,alpha=0,projection=OpticsConstants.PROJECTION_RECTILINEAR,
                               anglefactor=None,with_keys=False):
        ''' #17 Returns fisheye projection for focal length, incident angle, projection
            in case anglefactor is not supplied, default will be taken
            also returns lambda function(f,alpha,factor)
            http://pt4pano.com/de/blog/samyang-f2812mm-fullframe
            fisheye factor in PtGui https://www.ptgui.com/support.html#3_28           
            Projection Formulas:
            PROJECTION_RECTILINEAR   = f * Tan(alphaRad)
            PROJECTION_STEREOGRAPHIC = f * anglefactor * Tan(alphaRad / anglefactor)
            PROJECTION_EQUIDISTANT   = f * alphaRad
            PROJECTION_ORTHOGRAPHIC  = f * Sin(alphaRad)
            PROJECTION_EQUISOLID     = f * anglefactor * Sin(alphaRad / anglefactor)            
        '''    
        c,o,_ = OpticsCalculator.bootstrap()
        
        if not ( projection in c.PROJECTIONS ):
            print("projection not supported")
            return None
        
        factor = anglefactor         
        if factor == None:
            factor = c.PROJECTION_SPEC[projection][c.PROJECTION_FACTOR]
        image_projection = c.PROJECTION_SPEC[projection][c.PROJECTION_FUNCTION](f,alpha,factor) 
        params_in  = dict( zip( (c.FOCAL_LENGTH,c.INCIDENT_ANGLE,c.PROJECTION,c.ANGLE_FACTOR),
                                (f,alpha,projection,anglefactor) ) )        
        params_out = dict( zip( (c.ANGLE_FACTOR,c.IMAGE_PROJECTION),
                                (factor,image_projection) ) )
       
        result = o.get_results(result_dict=params_out,with_keys=with_keys,
                               tuple_name=c.PROJECTION,key_dict=params_in)
        return result 

    @staticmethod
    def get_fisheye_lens_projection(lens,alpha,with_keys=False):
        ''' #18 gets specific lens specs for a given fisheye
        '''
        c,o,_ = OpticsCalculator.bootstrap()
        if not ( lens in c.LENSES ):
            print("there'S no such lens")
            return None
        fisheye_lens_spec = c.FISHEYE_LENS_SPECS[lens]
        f = fisheye_lens_spec[c.FOCAL_LENGTH]
        projection = fisheye_lens_spec[c.PROJECTION_FUNCTION]
        factor = fisheye_lens_spec[c.PROJECTION_FACTOR] 
        image_projection = o.get_fisheye_projection(f,alpha,projection=projection,
                                                    anglefactor=factor,with_keys=with_keys)
        params_in  = dict( zip( (c.LENS,c.INCIDENT_ANGLE),
                                (lens,alpha) ) )        
        params_out = dict( zip( (c.FOCAL_LENGTH,c.PROJECTION,c.PROJECTION_FACTOR,c.IMAGE_PROJECTION),
                                (f,projection,factor,image_projection) ) )
      
        result = o.get_results(result_dict=params_out,with_keys=with_keys,
                               tuple_name=c.PROJECTION,key_dict=params_in)
        return result
    
    @staticmethod
    def get_cropped_resolution(megapixels=24,crop=0.5,with_keys=False):
        ''' #19 ImageCrop Functions: Cropped Image Size in Megapixels
        Cropped Megapixels. Let
        a) d = MP / A ( d: Pixel Density; MP: Megapixels; A:Area )
        b) A = w * h = R * h * h (width * height/R Image Ratio)
        c) w= crop * w; h= crop * h  (x:Image Crop factor 0..1 )
        d) A'= w* h= crop^2 * w * h = crop^2 * A
        e) d = MP / A = MP'/ A'=> MP'= MP * A'/ A = x^2 * MP
        '''
        c,o,_ = OpticsCalculator.bootstrap()
        cropped_resolution = crop**2 * megapixels
        params_in  = dict( zip( (c.PIXEL_NUMBER,c.CROP),
                                (megapixels,crop) ) )        
        params_out = dict( zip( (c.RESOLUTION),
                                (cropped_resolution) ) )
        
        result = o.get_results(result_dict=params_out,with_keys=with_keys,
                               tuple_name=c.RESOLUTION,key_dict=params_in)
        return result
    
    @staticmethod
    def get_crop_effective_focal_length(f=50,crop=0.5,with_keys=False):
        ''' #20 Image Crop Functions: Cropped effective focal length
            In essence the effective Focal length would result, if the
            cropped image would hit the ENTIRE sensor (instead of a section only)
            f: focal length / s:sensor length / s': cropped sensor length
            a) tan(alpha/2) = s / (2*f)   / original Relation
            b) tan(alpha'/2) = s'/ (2*f) / Geometry for Crop
            c) tan(alpha'/2) = s / (2*f') / "As If focal length f' " Crop would hit the complete sensor
            d) s' = x * s                / Crop Image
            b) c) d) combined
            tan(alpha'/2) = s' / (2*f) = s / (2*f') => f= (s/s') * f = f / crop
            e) f= f / x
        '''
        c,o,_ = OpticsCalculator.bootstrap()
        effective_crop_focal_length = f / crop

        params_in  = dict( zip( (c.FOCAL_LENGTH,c.CROP),
                                (f,crop) ) )        
        params_out = dict( zip( (c.CROP_FOCAL_LENGTH_EFFECTIVE),
                                (effective_crop_focal_length) ) )        
        result = o.get_results(result_dict=params_out, with_keys=with_keys,
                               tuple_name=c.CROP_FOCAL_LENGTH,key_dict=params_in)
        return result

    @staticmethod
    def get_equivalent_sensor_specs(f=50.,k=2,iso=100.,
                                    sensor=OpticsConstants.SENSOR_APSC,
                                    sensor_target=OpticsConstants.SENSOR_FF,
                                    with_keys=False):
        ''' ##21 Calculates equivalent lens specs for a given sensor type to a target sensor type
        '''
        c,o,_ = OpticsCalculator.bootstrap()
        specs_source = o.get_sensor_specs(sensor,with_keys=False)
        specs_target = o.get_sensor_specs(sensor_target,with_keys=False)
        crop_rel = specs_target[c.DIMENSION_DIAGONAL] / specs_source[c.DIMENSION_DIAGONAL]
        target = {c.CROP_RELATIVE:crop_rel,c.APERTURE_NUMBER:(crop_rel * k),
                  c.FOCAL_LENGTH:(crop_rel * f),c.ISO:(crop_rel**2 * iso)}
        params_in  = dict( zip( (c.FOCAL_LENGTH,c.APERTURE_NUMBER,c.ISO,c.SENSOR,c.SENSOR_TARGET),
                                (f,k,iso,sensor,sensor_target) ) )        
        # Round all items
        params_out = dict(map(lambda item: (item[0], round(item[1],1)), target.items()))

        result = o.get_results(result_dict=params_out,with_keys=with_keys,
                               tuple_name=c.EQUIVALENT_LENS_SPEC,key_dict=params_in)
        return result

    @staticmethod
    def get_astro_speed(f=50,k=2.8,sensor_type=OpticsConstants.SENSOR_FF,megapixels=24,
                        dimension=OpticsConstants.DIMENSION_WIDTH,with_keys=False):
        ''' #22 calculates pixel velocity due to earth rotation and astro exposure time
            according to 500/T rule and NPF Rule
            assumption: rotation equivalent to one day so camera speed is 
            24 hours = 360 degress -> 1hr = 15deg -> 1min = 0,24deg -> 1sec = 0.0041deg
            https://astrobackyard.com/the-500-rule/
            https://petapixel.com/2017/04/07/npf-rule-formula-sharp-star-photos-every-time/
        '''
        c,o,specs = OpticsCalculator.bootstrap(sensor_type=sensor_type,megapixels=megapixels)
        sensor_length = specs[dimension]
        direction = c.MAP_DIMENSION2DIRECTION[dimension]
        num_pixels = specs[c.MAP_DIMENSION2PIXELNUM[dimension]]
        fov = o.get_field_of_view(focal_length=f,sensor_type=sensor_type,with_keys=False)[direction]
        # calculate sensor length / pixels per degree [mm/degree]
        sensor_length_per_degree = sensor_length / fov
        pixels_per_degree = round( num_pixels / fov,0)
        crop = specs[c.DIMENSION_CROP]
        pixel_pitch = specs[c.DIMENSION_PIXEL_WIDTH]
        # max exposure time according to astro 500 rule
        astro500exposure = int(500 / (crop*f))
        # max exposure time according to astro NPF rule
        astroNPFexposure = ((35*k)+(30*pixel_pitch)) / f
        
        #  EARTH_ROTATION_SPEED [degrees/second] multiplied by [length/degrees] or [pixels/degrees]
        #  yields length/second or pixels/second
        length_per_second = sensor_length_per_degree * c.EARTH_ROTATION_SPEED
        pixels_per_second = pixels_per_degree * c.EARTH_ROTATION_SPEED
        
        # calculate spread of a point according to astro 500 and astro NPF rule
        length_astro500_exposure = length_per_second * astro500exposure
        pixels_astro500_exposure = pixels_per_second * astro500exposure
        length_astroNPF_exposure = length_per_second * astroNPFexposure
        pixels_astroNPF_exposure = pixels_per_second * astroNPFexposure
        
        result_keys = (c.DIMENSION,c.LENGTH,c.DIRECTION,c.PIXEL_NUMBER,c.FIELD_OF_VIEW,
                       c.LENGTH_PER_DEG,c.PIXELS_PER_DEG,c.LENGTH_PER_SEC,c.PIXELS_PER_SEC,c.CROP,
                       c.ASTRO_500_RULE,c.ASTRO_500_LENGTH,c.ASTRO_500_PIXEL,c.DIMENSION_PIXEL_WIDTH,
                       c.ASTRO_NPF_RULE,c.ASTRO_NPF_LENGTH,c.ASTRO_NPF_PIXEL)
        result = (dimension,sensor_length,direction,num_pixels,fov,
                  sensor_length_per_degree,pixels_per_degree,length_per_second,pixels_per_second,crop,
                  astro500exposure,length_astro500_exposure,pixels_astro500_exposure,pixel_pitch,
                  astroNPFexposure,length_astroNPF_exposure,pixels_astroNPF_exposure)

        params_out = dict(zip(result_keys,result))        
        params_in  = dict( zip( (c.FOCAL_LENGTH_CLOSEUP,c.APERTURE_NUMBER,c.SENSOR,c.PIXEL_NUMBER,c.DIMENSION),
                                (f,k,sensor_type,megapixels,dimension) ) )        
    
        result = o.get_results(result_dict=params_out,with_keys=with_keys,
                               tuple_name=c.ASTRO_SPEED,key_dict=params_in)
        return result

# Further IDeas
# calculation: resolution dpi, can the eye discern it? 
# https://de.wikipedia.org/wiki/Sehsch%C3%A4rfe
# Resolution eye ~ 2  angle minute = 2* 360° / (24 * 60 )  = 0.3 deg   visus 2
# Visus 1' / individual resolution    
# https://de.wikipedia.org/wiki/Sehsch%C3%A4rfe