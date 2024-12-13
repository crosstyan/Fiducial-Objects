# APPLICATIONS

## DoducoPose

**Process video:**
- _Cucomarker_:
    - ./processPose ../DATA/TestGroups/TestGroup_1/Videos/cameraparams.yml 0.074 ../DATA/TestGroups/TestGroup_1/Videos/test4_map.yml --markerParams=../DATA/TestGroups/TestGroup_1/Markers/pentagonConfig.yml --v --vdebug --vframe --inputVideo=../DATA/TestGroups/TestGroup_1/Videos/test4.mp4
    - ./processPose ../DATA/TestGroups/TestGroup_3/Video/cameraparams.yml 0.024 ../DATA/TestGroups/TestGroup_3/Video/world.yml --markerParams=../DATA/TestGroups/TestGroup_3/Markers/example.yml --v --vdebug --vframe --inputVideo=../DATA/TestGroups/TestGroup_3/Video/mapCreation.mp4
    - ./processPose ../DATA/TestGroups/TestGroup_4/Videos/cameraparams.yml 0.024 ../DATA/TestGroups/TestGroup_4/Videos/world.yml --markerParams=../DATA/TestGroups/TestGroup_4/Markers/example.yml3 --v --vdebug --vframe --inputVideo=../DATA/TestGroups/TestGroup_4/Videos/test1.mp4
    - ./processPose ../DATA/TestGroups/TestGroup_5/Videos/cameraparams.yml 0.024 ../DATA/TestGroups/TestGroup_5/Videos/world.yml --markerParams=../DATA/TestGroups/TestGroup_5/Markers/example.yml --v --vdebug --vframe --inputVideo=../DATA/TestGroups/TestGroup_5/Videos/test1.mp4
    - ./processPose ../DATA/TestGroups/TestGroup_6/Videos/cameraparams.yml 0.0355 ../DATA/TestGroups/TestGroup_6/Videos/world.yml --markerParams=../DATA/TestGroups/TestGroup_6/Markers/example.yml --v --vdebug --vframe --inputVideo=../DATA/TestGroups/TestGroup_6/Videos/mapCreation.mp4
    - ./processPose ../../../../GlobalDATA/OptitrackVideos/camParams.yml 0.024 ../DATA/TestGroups/TestGroup_8/Videos/world.yml --markerParams=../DATA/TestGroups/TestGroup_8/Markers/example.yml --v --vdebug --vframe --inputVideo=../../../../GlobalDATA/OptitrackVideos/Group_1/02/cam0.avi
    - ./processPose ../../../../GlobalDATA/OptitrackVideos/camParams.yml 0.024 ../DATA/TestGroups/TestGroup_8/Videos/world.yml --markerParams=../DATA/TestGroups/TestGroup_8/Markers/example.yml --v --vdebug --vframe --generatePosesOutputFile --outputPosePath=2_poses_r40_d3.txt --thresAngle=40 --thresNDet=3 --inputVideo=../../../../GlobalDATA/OptitrackVideos/Group_2/02/cam_0.avi
    - ./processPose ../DATA/TestGroups/TestGroup_10/Videos/cameraparams.yml 0.035 ../DATA/TestGroups/TestGroup_10/Videos/world.yml --markerParams=../DATA/TestGroups/TestGroup_10/Markers/markers.yml --v --vdebug --vframe --generatePosesOutputFile --outputPosePath=0_0.txt --thresAngle=30 --thresNDet=3 --idsRange=1-6 --inputVideo=../DATA/TestGroups/TestGroup_10/Videos/test5.mp4
    - ./processPose ../../../../GlobalDATA/OptitrackVideos/camParams.yml 0.035 ../DATA/TestGroups/TestGroup_10/Videos/world.yml --markerParams=../DATA/TestGroups/TestGroup_10/Markers/markers.yml --v --vdebug --vframe --generatePosesOutputFile --outputPosePath=0_0.txt --idsRange=1-12 --inputVideo=../../../../GlobalDATA/Recorder/61cm/40cm/cam0.avi
    - ./processPose ../../../../GlobalDATA/OptitrackVideos/camParams.yml 0.035 ../DATA/TestGroups/TestGroup_10/Videos/world.yml --markerParams=../DATA/TestGroups/TestGroup_10/Markers/markers.yml --v --vdebug --vframe --generatePosesOutputFile --outputPosePath=0_0.txt --idsRange=1-12 --framesRanges=../../../../GlobalDATA/Recorder/61cm/40cm/frames.txt --solvePNPMethod=1 --inputVideo=../../../../GlobalDATA/Recorder/61cm/40cm/cam0.avi
    - ./processPose ../DATA/TestGroups/TestGroup_11/Videos/cameraparams.yml 0.0221876 ../DATA/TestGroups/TestGroup_11/Videos/world.yml --markerParams=../DATA/TestGroups/TestGroup_11/Markers/markers.yml --v --vdebug --vframe --inputVideo=../DATA/TestGroups/TestGroup_11/Videos/test6.mp4
    - ./processPose ../DATA/Cameras/mobileCameraCalibration.yml 0.0221876 ../DATA/TestGroups/TestGroup_11/Videos/world.yml --markerParams=../DATA/TestGroups/TestGroup_11/Markers/markers.yml --v --vdebug --vframe --inputVideo=../DATA/TestGroups/TestGroup_11/Videos/test6.mp4
    - ./processPose ../DATA/Cameras/mobileCameraCalibration.yml 0.0221876 ../DATA/TestGroups/TestGroup_11/Videos/world.yml --markerParams=../DATA/TestGroups/TestGroup_11/Markers/markers.yml --v --vdebug --vframe --inputImage=../DATA/TestGroups/TestGroup_11/Videos/image.png
    - ./processPose ../DATA/Cameras/mobileCameraCalibration.yml 0.0221876 ../DATA/TestGroups/TestGroup_10/Videos/world.yml --markerParams=../DATA/TestGroups/TestGroup_10/Markers/markers.yml --v --vdebug --vframe --inputVideo=/home/paul/Workspace/DoducoPose/0_Workspace/DoducoPose/DoducoPose_DATA/Polyhucos/P_001/Map/video.mp4
        - ./processPose ../DATA/Cameras/mobileCameraCalibration.yml 0.0221876 ../DATA/TestGroups/TestGroup_10/Videos/worldMarkerMapper.yml --markerParams=../DATA/TestGroups/TestGroup_10/Markers/markers.yml --v --vdebug --vframe --inputVideo=/home/paul/Workspace/DoducoPose/0_Workspace/DoducoPose/DoducoPose_DATA/Polyhucos/P_001/Map/video.mp4



    - ./processPose ../DATA/Cameras/camera4KID3.yml 0.015 ../DATA/TestGroups/TestGroup_14_RhombicDodecahedron/world.yml --markerParams=../DATA/TestGroups/TestGroup_14_RhombicDodecahedron/markers.yml --v --vdebug --vframe --inputVideo=../DATA/TestGroups/TestGroup_14_RhombicDodecahedron/video.avi --generatePosesOutputFile --outputPosePath=rhombicPoses.txt --detectionMode=1 --distancesOutputPath=distances.txt

    _Only 10 frames_:
    - ./processPose ../DATA/Cameras/camera4KID3.yml 0.015 ../DATA/TestGroups/TestGroup_14_RhombicDodecahedron/world.yml --markerParams=../DATA/TestGroups/TestGroup_14_RhombicDodecahedron/markers.yml --v --vdebug --vframe --inputVideo=../DATA/TestGroups/TestGroup_14_RhombicDodecahedron/1_6_Rhombic.avi --generatePosesOutputFile --outputPosePath=rhombicPoses.txt --detectionMode=1 --distancesOutputPath=distances.txt
        - ./processPose ../DATA/Cameras/camera4KID3.yml 0.015 ../DATA/TestGroups/TestGroup_14_RhombicDodecahedron/world.yml --markerParams=../DATA/TestGroups/TestGroup_14_RhombicDodecahedron/markers.yml --v --vdebug --vframe --inputVideo=../DATA/TestGroups/TestGroup_14_RhombicDodecahedron/1_6_Rhombic.avi --generatePosesOutputFile --outputPosePath=rhombicPoses.txt --detectionMode=1 --distancesOutputPath=distances.txt --applyLookedMarkers


    **__Cube__**
    - ./processPose ../DATA/Cameras/cameraSetSame.yml 0.015 ../DATA/TestGroups/TG_17/map.yml --markerParams=../DATA/TestGroups/TG_17/config.yml --v --vdebug --vframe --inputVideo=../DATA/TestGroups/TG_17/video.avi --generatePosesOutputFile --outputPosePath=cubePoses.txt --detectionMode=1
    
    **__Prism__**
    - ./processPose ../DATA/Cameras/camera4KID2New.yml 0.015 ../DATA/TestGroups/TG_16/map.yml --markerParams=../DATA/TestGroups/TG_16/configHexagon.yml,../DATA/TestGroups/TG_16/configRectangle.yml --v --vdebug --vframe --inputVideo=../DATA/TestGroups/TG_16/video.avi --generatePosesOutputFile --outputPosePath=prismPoses.txt --detectionMode=1

    **__Dodecahedron__**
    - ./processPose ../DATA/Cameras/camera4KID2New.yml 0.015 ../DATA/TestGroups/TG_15/map.yml --markerParams=../DATA/TestGroups/TG_15/config.yml --v --vdebug --vframe --inputVideo=../DATA/TestGroups/TG_15/video.avi --generatePosesOutputFile --outputPosePath=dodecPoses.txt --detectionMode=1 
        _Refined_
        - ./processPose ../DATA/Cameras/camera4KID2New.yml 0.015 ../DATA/TestGroups/TG_15/map.yml --markerParams=../DATA/TestGroups/TG_15/config.yml --v --vdebug --vframe --inputVideo=../DATA/TestGroups/TG_15/video.avi --generatePosesOutputFile --outputPosePath=dodecPoses.txt --detectionMode=1 --applyLookedMarkers --thresAngle=15 
        _Scaled_
        - ./processPose ../DATA/Cameras/camera4KID2New.yml 0.015 ../DATA/TestGroups/TG_15/mapScaled.yml --markerParams=../DATA/TestGroups/TG_15/config.yml --v --vdebug --vframe --inputVideo=../DATA/TestGroups/TG_15/video.avi --generatePosesOutputFile --outputPosePath=dodecPosesScaled.txt --detectionMode=1 
        _Scale-factor_
        ./processPose ../DATA/Cameras/camera4KID2New.yml 0.015 ../DATA/TestGroups/TG_15/map.yml --markerParams=../DATA/TestGroups/TG_15/config.yml --v --vdebug --vframe --inputVideo=../DATA/TestGroups/TG_15/video.avi --generatePosesOutputFile --outputPosePath=scaledDodecPoses.txt --detectionMode=1 --distancesOutputPath=scaledDodecDistances.txt --scaleForMap=0.012702    


    **__Sets_Dodecahedron__** <!-- NO INTERESA: Cada set tiene parametros de camara diferentes + Alineamiento con optitrack no es posible debido a cambio en centro de objeto -->
    - ./processPose ../DATA/Cameras/camera4KID2New.yml 0.015 ../DATA/TestGroups/TG_15/map.yml --markerParams=../DATA/TestGroups/TG_15/config.yml --v --vdebug --vframe --inputVideo=../DATA/TestGroups/TG_15/setsVideo.avi --generatePosesOutputFile --outputPosePath=dodecPosesSets.txt --detectionMode=1
    
    
- _Aruco_:
    - ./processPose ../DATA/TestGroups/TestGroup_2/cameraparams.yml 0.024 ../DATA/TestGroups/TestGroup_2/world.yml --v --vdebug --vframe --detectionMode=2 --inputVideo=../DATA/TestGroups/TestGroup_2/testVideo.mp4
    - ./processPose ../DATA/TestGroups/Aruco/ArucoCube_Big/Videos/cameraparams.yml 0.045 ../DATA/TestGroups/Aruco/ArucoCube_Big/Videos/world.yml --v --vdebug --vframe --detectionMode=2 --inputVideo=../DATA/TestGroups/Aruco/ArucoCube_Big/Videos/arucoBig.mp4
    - ./processPose ../../../../GlobalDATA/OptitrackVideos/camParams.yml 0.045 ../DATA/TestGroups/Aruco/ArucoCube_Big/Videos/world.yml --v --vdebug --vframe --generatePosesOutputFile --outputPosePath=0_0_big.txt --thresAngle=30 --thresNDet=3 --detectionMode=2 --inputVideo=../../../../GlobalDATA/OptitrackVideos/Group_5/00/cam0.avi
    - ./processPose ../../../../GlobalDATA/OptitrackVideos/camParams.yml 0.024 ../DATA/TestGroups/Aruco/ArucoCube_Small/Videos/world.yml --v --vdebug --vframe --generatePosesOutputFile --outputPosePath=0_0_small.txt --thresAngle=30 --thresNDet=3 --detectionMode=2 --inputVideo=../../../../GlobalDATA/OptitrackVideos/Group_5/01/cam0.avi
    - ./processPose ../DATA/TestGroups/TestGroup_12/Videos/cameraparams.yml 0.045909 ../DATA/TestGroups/TestGroup_12/Videos/world.yml --v --vdebug --vframe --generatePosesOutputFile --outputPosePath=0_0.txt --thresAngle=0 --thresNDet=0 --detectionMode=2 --inputVideo=../DATA/TestGroups/TestGroup_12/Videos/test.mp4
        - ./processPose ../DATA/TestGroups/TestGroup_12/Videos/cameraparams.yml 0.045909 ../DATA/TestGroups/TestGroup_12/Videos/world.yml --v --vdebug --vframe --generatePosesOutputFile --outputPosePath=0_0.txt --thresAngle=0 --thresNDet=0 --detectionMode=2 --inputVideo=../../../../DoducoPose_externalSoftware/Recorder/recorder/build/OUT/circle_80cm/CubeBig/cam0.avi
        - ./processPose ../DATA/TestGroups/TestGroup_12/Videos/cameraparams.yml 0.045909 ../DATA/TestGroups/TestGroup_12/Videos/world.yml --v --vdebug --vframe --generatePosesOutputFile --outputPosePath=0_0.txt --thresAngle=0 --thresNDet=0 --detectionMode=2 --inputVideo=../../../../DoducoPose_externalSoftware/Recorder/recorder/build/OUT/circle_40cm/CubeBig/cam0.avi
    
    - ./processPose ../../../../GlobalDATA/OptitrackVideos/camParams.yml 0.045909 ../DATA/TestGroups/TestGroup_12/Videos/world.yml --v --vdebug --vframe --generatePosesOutputFile --outputPosePath=0_0.txt --detectionMode=2 --idsRange=1-6 --framesRanges=../../../../GlobalDATA/Recorder/61cm/40cm/frames.txt --inputVideo=../../../../GlobalDATA/Recorder/61cm/40cm/cam0.avi
    
    - ./processPose ../DATA/Cameras/mobileCameraCalibration.yml 0.036726 ../DATA/TestGroups/TestGroup_13/Videos/world.yml --detectionMode=2 --v --vdebug --vframe --inputVideo=../DATA/TestGroups/TestGroup_13/Videos/test.mp4
        _ReSLAM_
        - ./processPose ../DATA/Cameras/mobileCameraCalibration.yml 0.036726 ../DATA/TestGroups/TestGroup_13/Videos/MapsReslam/ConKeypoints/world.yml --detectionMode=2 --v --vdebug --vframe --inputVideo=../DATA/TestGroups/TestGroup_13/Videos/test.mp4
        - ./processPose ../DATA/Cameras/mobileCameraCalibration.yml 0.036726 ../DATA/TestGroups/TestGroup_13/Videos/MapsReslam/SinKeypoints/world.yml --detectionMode=2 --v --vdebug --vframe --inputVideo=../DATA/TestGroups/TestGroup_13/Videos/test.mp4
        _UcoSLAM_
        - ./processPose ../DATA/Cameras/mobileCameraCalibration.yml 0.036726 ../DATA/TestGroups/TestGroup_13/Videos/MapsUcoslam/ConKeypoints/world.yml --detectionMode=2 --v --vdebug --vframe --inputVideo=../DATA/TestGroups/TestGroup_13/Videos/test.mp4
        - ./processPose ../DATA/Cameras/mobileCameraCalibration.yml 0.036726 ../DATA/TestGroups/TestGroup_13/Videos/MapsUcoslam/SinKeypoints/world.yml --detectionMode=2 --v --vdebug --vframe --inputVideo=../DATA/TestGroups/TestGroup_13/Videos/test.mp4
        _MarkerMapper_
        - ./processPose ../DATA/Cameras/mobileCameraCalibration.yml 0.036726 ../DATA/TestGroups/TestGroup_13/Videos/MapMarkerMapper/world.yml --detectionMode=2 --v --vdebug --vframe --inputVideo=../DATA/TestGroups/TestGroup_13/Videos/test.mp4
    ./processPose ../DATA/Cameras/camera4KID3.yml 1 ../DATA/TestGroups/TestGroup_12/Videos/MarkerMapper/arucocubebig.yml --detectionMode=2 --v --vdebug --vframe --inputImage=../DATA/TestGroups/TestGroup_12/Videos/MarkerMapper/images/image01.jpg
    - ./processPose ../DATA/Cameras/camera4KID3.yml 1 ../DATA/TestGroups/TestGroup_12/Videos/MarkerMapper/arucocubebig.yml --detectionMode=2 --v --vdebug --vframe --inputVideo=../DATA/TestGroups/TestGroup_12/Videos/MarkerMapper/test.mkv
    - ./processPose ../DATA/Cameras/streamCamLogiCalibration.yml 0.036726 ../DATA/TestGroups/TestGroup_13/Videos/MarkerMapperNew/world.yml --detectionMode=2 --v --vdebug --vframe --inputVideo=../DATA/TestGroups/TestGroup_13/Videos/my_video-3.mkv
    - ./processPose ../DATA/Cameras/streamCamLogiCalibration.yml 0.036726 ../DATA/TestGroups/TestGroup_13/Videos/MarkerMapperNew/world.yml --detectionMode=2 --v --vdebug --vframe --inputVideo=../DATA/TestGroups/TestGroup_13/Videos/my_video-3.mkv


- _Calibration_:
    - ./processPose ../../Calibration/Test/Video2/cameraparams.yml 0.0431 ../../Calibration/Test/Video2/world.yml --v --vdebug --vframe --generatePosesOutputFile --outputPosePath=0_0.txt --detectionMode=2 --idsRange=1-16 --inputVideo=../../Calibration/Test/Video2/calibVideo.mp4

**Wand painter:**
- _wandPainter_:s.yml 0.024 ../DATA/TestGroups/TestGroup_5/Markers/example.yml ../DATA/TestGroups/TestGroup_5/Videos/world.yml 0.0355 ../DATA/TestGroups/TestGroup_6/Markers/example.yml ../DATA/TestGroups/TestGroup_6/Videos/world.yml --cornersConfig=savedCorners.yml
    - ./wandPainter --cameraId=2 ../DATA/laborator
    - ./wandPainter --cameraId=2 ../DATA/laboratorioCamParams.yml 0.024 ../DATA/TestGroups/TestGroup_5/Markers/example.yml ../DATA/TestGroups/TestGroup_5/Videos/world.yml 0.0355 ../DATA/TestGroups/TestGroup_6/Markers/example.yml ../DATA/TestGroups/TestGroup_6/Videos/world.yml
    - ./wandPainter --cameraId=2 ../DATA/laboratorioCamParamioCamParams.yml 0.024 ../DATA/TestGroups/TestGroup_5/Markers/example.yml ../DATA/TestGroups/TestGroup_5/Videos/world.yml 0.0355 ../DATA/TestGroups/TestGroup_6/Markers/example.yml ../DATA/TestGroups/TestGroup_6/Videos/world.yml --markedPlane --continuousMarkedPlaneSearching
    - ./wandPainter --inputVideoPath=../DATA/TestGroups/TestGroup_7/shorted.avi ../DATA/TestGroups/TestGroup_7/laboratorioCamParams.yml 0.024 ../DATA/TestGroups/TestGroup_5/Markers/example.yml ../DATA/TestGroups/TestGroup_5/Videos/world.yml 0.0355 ../DATA/TestGroups/TestGroup_6/Markers/example.yml ../DATA/TestGroups/TestGroup_6/Videos/world.yml --interpolation --generateOutputVideo
    - ./wandPainter --cameraId=2 ../DATA/Cameras/streamCamLogiCalibration.yml 0.0221876 ../DATA/TestGroups/TestGroup_11/Markers/markers.yml ../DATA/TestGroups/TestGroup_11/Videos/world.yml --calibratedWandFile=calibWand.yml --cornersConfig=savedCorners.yml --interpolation --generateOutputVideo
    - ./wandPainter --cameraId=0 ../DATA/Cameras/camera4KID3.yml 0.0221876 ../DATA/TestGroups/TestGroup_11/Markers/markers.yml ../DATA/TestGroups/TestGroup_11/Videos/world.yml --calibratedWandFile=calibWand.yml --interpolation --generateOutputVideo
    - ./wandPainter --cameraId=2 ../DATA/Cameras/streamCamLogiCalibration.yml 0.0221876 ../DATA/TestGroups/TestGroup_11/Markers/markers.yml ../DATA/TestGroups/TestGroup_11/Videos/world.yml --calibratedWandFile=calibWand.yml --interpolation --generateOutputVideo --cornersConfig=savedCorners.yml
    - ./wandPainter --cameraId=2 ../DATA/Cameras/streamCamLogiCalibration.yml 0.036726 ../DATA/TestGroups/TestGroup_11/Markers/markers.yml ../DATA/TestGroups/TestGroup_13/Videos/MarkerMapperNew/map.yml 0.0431 ../../../DoducoPose_DATA/Calibration/Map/world.yml --interpolation --generateOutputVideo --cornersConfig=savedCorners.yml
        - ./wandPainter --cameraId=2 ../DATA/Cameras/streamCamLogiCalibration.yml 0.036726 ../DATA/TestGroups/TestGroup_11/Markers/markers.yml ../DATA/TestGroups/TestGroup_13/Videos/MarkerMapperNew/map.yml 0.0431 ../../../DoducoPose_DATA/Calibration/Map/world.yml --interpolation --generateOutputVideo --cornersConfig=savedCorners.yml



**Calibrate wand:**
- ./calibrateWand --cameraId=0 ../DATA/Cameras/ownCameraCalibration.yml 0.0221876 ../DATA/TestGroups/TestGroup_11/Markers/markers.yml ../DATA/TestGroups/TestGroup_11/Videos/world.yml 0.0431 ../../../DoducoPose_DATA/Calibration/Map/world.yml
- ./calibrateWand --cameraId=2 ../DATA/Cameras/streamCamLogiCalibration.yml 0.0221876 ../DATA/TestGroups/TestGroup_11/Markers/markers.yml ../DATA/TestGroups/TestGroup_11/Videos/world.yml 0.0431 ../../../DoducoPose_DATA/Calibration/Map/world.yml
- ./calibrateWand --cameraId=2 ../DATA/Cameras/camera4KID3.yml 0.0221876 ../DATA/TestGroups/TestGroup_11/Markers/markers.yml ../DATA/TestGroups/TestGroup_11/Videos/world.yml 0.0431 ../../../DoducoPose_DATA/Calibration/Map/world.yml
- ./calibrateWand --cameraId=0 ../DATA/Cameras/720SolapaCalibration.yml 0.0221876 ../DATA/TestGroups/TestGroup_11/Markers/markers.yml ../DATA/TestGroups/TestGroup_11/Videos/world.yml 0.0431 ../../../DoducoPose_DATA/Calibration/Map/world.yml
- ./calibrateWand --cameraId=2 ../DATA/Cameras/streamCamLogiCalibration.yml 0.036726 ../DATA/TestGroups/TestGroup_11/Markers/markers.yml ../DATA/TestGroups/TestGroup_13/Videos/MarkerMapperNew/world.yml 0.0431 ../../../DoducoPose_DATA/Calibration/Map/world.yml



---



# TESTS

## Third parties

**Cucomarker:**
- ./detectionTest ../../../../GlobalDATA/OptitrackVideos/Group_4/2/cam0.avi ../DATA/TestGroups/TestGroup_4/Markers/example.yml
- ./detectionTest ../DATA/TestGroups/TestGroup_10/Videos/test.mp4 ../DATA/TestGroups/TestGroup_10/Markers/markers.yml
    - ./detectionTest ../DATA/TestGroups/TestGroup_10/Videos/testt.avi ../DATA/TestGroups/TestGroup_10/Markers/markers.yml
- ./detectionTest ../DATA/TestGroups/TestGroup_11/Videos/test.mp4 ../DATA/TestGroups/TestGroup_11/Markers/markers.yml
