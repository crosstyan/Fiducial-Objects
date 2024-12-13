**Video:**
./utils_developer/processPose ../DATA/Cube/Scales/cameraCalibration.yml 0.0342 ../DATA/Cube/Scales/map.yml --markerParams=../DATA/Cube/Scales/config.yml --v --vframe --inputVideo=../DATA/Cube/Scales/9_2_Cube.avi --detectionMode=1 --generatePosesOutputFile --outputPosePath=cubePoses.txt --generateDistancesOutputFile --outputDistancePath=cubeDistances.txt --thresAngle=15 --thresDiff=5 --scaleForMap=0.036727 --startingPaused --applyRefinement

**Image:**
./utils_developer/processPose ../DATA/Dodec/cameraCalibration.yml 0.0342 ../DATA/Dodec/map.yml --markerParams=../DATA/Dodec/config.yml --v --vframe --inputImage=../DATA/Dodec/frame2.png --detectionMode=1 --generatePosesOutputFile --outputPosePath=dodecPoses.txt --generateDistancesOutputFile --outputDistancePath=dodecDistances.txt --thresAngle=15 --thresDiff=5 --scaleForMap=0.036727 --startingPaused --applyRefinement
