#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    //ofSetLogLevel(OF_LOG_VERBOSE);
    
    // enable depth->video image calibration
    kinect.setRegistration(true);
    
    kinect.init();
    //kinect.init(true); // shows infrared instead of RGB video image
    //kinect.init(false, false); // disable video image (faster fps)
    
    kinect.open();		// opens first available kinect
    //kinect.open(1);	// open a kinect by id, starting with 0 (sorted by serial # lexicographically))
    //kinect.open("A00362A08602047A");	// open a kinect using it's unique serial #
    
    // print the intrinsic IR sensor values
    if(kinect.isConnected()) {
        ofLogNotice() << "sensor-emitter dist: " << kinect.getSensorEmitterDistance() << "cm";
        ofLogNotice() << "sensor-camera dist:  " << kinect.getSensorCameraDistance() << "cm";
        ofLogNotice() << "zero plane pixel size: " << kinect.getZeroPlanePixelSize() << "mm";
        ofLogNotice() << "zero plane dist: " << kinect.getZeroPlaneDistance() << "mm";
    }
    
    ofSetFrameRate(60);
    
    glPointSize(3);
    
    ofMesh newMesh;
    totalMesh.push_back(newMesh);
    
    w = 640;
    h = 480;
    
    minFigureDistance = 400;
    maxFigureDistance = 1300;
    
    lastSampleLoc[0] = rand() % w;
    lastSampleLoc[1] = rand() % h;
    nearby = 20;
    minConnectionDistance = 10;
    maxConnectionDistance = 30;

}

//--------------------------------------------------------------
void ofApp::update(){
    kinect.update();
}

//--------------------------------------------------------------
void ofApp::draw(){
    ofSetColor(255, 255, 255);
    
    easyCam.begin();
    drawPointCloud();
    easyCam.end();
    
    //kinect.drawDepth(10, 10, 400, 300);
}

void ofApp::drawPointCloud() {
    ofMesh *currentMesh = &totalMesh[totalMesh.size()-1];
    
    currentMesh->setMode(OF_PRIMITIVE_POINTS);
    
    
    
    //sample a new location
    int *x = &lastSampleLoc[0];
    int *y = &lastSampleLoc[1];
    if(kinect.getDistanceAt(*x,*y) > minFigureDistance && kinect.getDistanceAt(*x,*y) < maxFigureDistance) {
        
        currentMesh->addColor(kinect.getColorAt(*x,*y));
        currentMesh->addVertex(kinect.getWorldCoordinateAt(*x, *y));
    }
    else {
        lastSampleLoc[0] = rand() % w;
        lastSampleLoc[1] = rand() % h;
    }
    
    *x += ofRandom((-1*nearby), nearby);
    *y += ofRandom((-1*nearby), nearby);
    
    //draw each point every frame
    ofPushMatrix();
    ofScale(.8, -.8, -.8);
    ofTranslate(0, 0, -1000);
    ofEnableDepthTest();
    
    for(int i = 0; i < totalMesh.size(); i++){
        totalMesh[i].drawVertices();
    }
    
    ofDisableDepthTest();
    ofPopMatrix();
    
    //connect the lines every second
    float currentSec = ofGetElapsedTimef();
    if(currentSec> lastSecond+1){
        lastSecond = currentSec;
        
        currentMesh->setMode(OF_PRIMITIVE_LINES);
        
        int numVerts = currentMesh->getNumVertices();
        for (int a=0; a<numVerts; ++a) {
            ofVec3f verta = currentMesh->getVertex(a);
            for (int b=a+1; b<numVerts; ++b) {
                ofVec3f vertb = currentMesh->getVertex(b);
                float distance = verta.distance(vertb);
                if (distance <= maxConnectionDistance && distance >= minConnectionDistance) {
                    currentMesh->addIndex(a);
                    currentMesh->addIndex(b);
                }
            }
        }
        
        lastSampleLoc[0] = rand() % w;
        lastSampleLoc[1] = rand() % h;
        
        ofMesh mesh;
        totalMesh.push_back(mesh);
        
    }
    
}

//--------------------------------------------------------------
void ofApp::exit() {
    kinect.setCameraTiltAngle(0); // zero the tilt on exit
    kinect.close();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){
    
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){
    
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){
    
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){
    
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){
    
}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){
    
}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){
    
}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){
    
}
