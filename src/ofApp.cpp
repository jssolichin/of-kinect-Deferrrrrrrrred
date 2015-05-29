#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    
    //---EDITABLES
    //OPENCV Threshold
    nearThreshold = 0;
    farThreshold  = 32;
    minBlobSize = (kinect.width*kinect.height)/16;
    maxBlobSize = (kinect.width*kinect.height)/8;
    
    //MyPointCloud Threshold
    w = 640;
    h = 480;
    nearby = 20;
    minConnectionDistance = 10;
    maxConnectionDistance = 30;
    minFigureDistance = 400;
    maxFigureDistance = 1300;
    lastSampleLoc[0] = rand() % w;
    lastSampleLoc[1] = rand() % h;
    
    //---SETUP
    
    //ofSetLogLevel(OF_LOG_VERBOSE);
    
    // enable depth->video image calibration
    kinect.setRegistration(true);
    
    kinect.init();
    //kinect.init(true); // shows infrared instead of RGB video image
    //kinect.init(false, false); // disable video image (faster fps)
    
    kinect.open();		// opens first available kinect
    
    // print the intrinsic IR sensor values
    if(kinect.isConnected()) {
        ofLogNotice() << "sensor-emitter dist: " << kinect.getSensorEmitterDistance() << "cm";
        ofLogNotice() << "sensor-camera dist:  " << kinect.getSensorCameraDistance() << "cm";
        ofLogNotice() << "zero plane pixel size: " << kinect.getZeroPlanePixelSize() << "mm";
        ofLogNotice() << "zero plane dist: " << kinect.getZeroPlaneDistance() << "mm";
    }
    
    kinect.setCameraTiltAngle(20);
    
    depthImage.allocate(kinect.width, kinect.height);
    grayImage.allocate(kinect.width, kinect.height);
    colorImage.allocate(kinect.width, kinect.height);
    redImage.allocate(kinect.width, kinect.height);

    ofSetFrameRate(60);
    
    glPointSize(3);
    
    ofMesh newMesh;
    totalMesh.push_back(newMesh);
    
    kinect.enableDepthNearValueWhite(false);
    
    //finder.setup("haarcascade_frontalface_default.xml");
    //img.loadImage("test.jpg");
    
    
}

//--------------------------------------------------------------
void ofApp::update(){
    
    kinect.update();
    
    /** DEBUG **/
    
    //depthImage.setFromPixels(pix, kinect.width, kinect.height);
    //depthImage.flagImageChanged();
    
    grayImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);
    grayImage.mirror(false, true);
    
    unsigned char * pix = grayImage.getPixels();
    int numPixels = grayImage.getWidth() * grayImage.getHeight()-1;
    
    colorImage.setFromPixels(kinect.getPixels(), kinect.width, kinect.height);
    colorImage.mirror(false, true);
    colorImage.convertToGrayscalePlanarImage(redImage, 0);
    
    for(int i = numPixels; i > 0 ; i--){
        if( pix[i] > nearThreshold && pix[i] < farThreshold ){
            pix[i] = 255;
        }else{
            pix[i] = 0;
        }
    }
    
    //update the cv image
    grayImage.flagImageChanged();

    
    contourFinder.findContours(grayImage, minBlobSize, maxBlobSize, 2, false);
    if(contourFinder.nBlobs >= 2){
        ofxCvBlob blob0 = contourFinder.blobs.at(0);
        ofxCvBlob blob1 = contourFinder.blobs.at(1);
        
        /*
         finder.findHaarObjects(redImage);
         if(finder.blobs.size() >= 2){
         
         ofxCvBlob blob0 = finder.blobs[0];
         ofxCvBlob blob1 = finder.blobs[1];
         */
        
        blobQ = twoToQ(blob0, blob1);
        
    }
    
    
}

//--------------------------------------------------------------
void ofApp::draw(){
    ofSetColor(255, 255, 255);
    
    
    /** DEBUG **/
    
    //depthImage.draw(15, 15, 400, 300);
    redImage.draw(425, 15, 400, 300);
    grayImage.draw(15, 325, 400, 300);
    contourFinder.draw(15, 325, 400, 300);
    
    
    char reportStr[1024];
    sprintf(reportStr, "set near threshold %i (press: + -)\nset far threshold %i (press: < >)\nnum blobs found %i, fps: %i", nearThreshold, farThreshold, (int)contourFinder.blobs.size(), (int)ofGetFrameRate());
    ofDrawBitmapString(reportStr, 20, 650);
    
    /** App **/
    
    /*
    //find face blobs
    for(unsigned int i = 0; i < finder.blobs.size(); i++) {
        ofRectangle cur = finder.blobs[i].boundingRect;
        ofRect(cur.x, cur.y, cur.width, cur.height);
    }
    */
    
    easyCam.begin();
    
    ofPushMatrix();
    
    blobQ.getRotate(blobAngle, blobAxis.x, blobAxis.y, blobAxis.z);
    
    ofRotate(blobAngle, blobAxis.x, blobAxis.y, blobAxis.z);
    
    ofSetColor(0,0,255);
    ofNoFill();
    ofDrawBox(0,0, 0, 50, 50, 100);
    
    //drawPointCloud();
    drawPointCloudAll();
    
    ofPopMatrix();
    
    easyCam.end();
    
    
}

ofQuaternion ofApp::qFromV(ofVec3f v, ofVec3f u)
{
    ofQuaternion q;
    float cos_theta = u.normalize().dot(v.normalize());
    float angle = acos(cos_theta)*100;
    ofVec3f w = u.cross(v).normalize();
    
    //ofLog() << cos_theta << " " << angle << " " << w;
    q.makeRotate(angle, w);
    
    return q;
}

ofQuaternion ofApp::twoToQ(ofxCvBlob blob0, ofxCvBlob blob1)
{
    ofQuaternion q;
    
    int x0 = (int)blob0.centroid.x;
    int y0 = (int)blob0.centroid.y;
    
    int x1 = (int)blob1.centroid.x;
    int y1 = (int)blob1.centroid.y;
    
    ofVec3f one = kinect.getWorldCoordinateAt(x0, y0);
    ofVec3f two = kinect.getWorldCoordinateAt(x1, y1);
    
    //ofQuaternion q;
    q = qFromV(one,two);
    
    return q;
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


void ofApp::drawPointCloudAll() {
    int w = 640;
    int h = 480;
    ofMesh mesh;
    mesh.setMode(OF_PRIMITIVE_POINTS);
    int step = 2;
    for(int y = 0; y < h; y += step) {
        for(int x = 0; x < w; x += step) {
            if(kinect.getDistanceAt(x, y) > 0) {
                mesh.addColor(kinect.getColorAt(x,y));
                mesh.addVertex(kinect.getWorldCoordinateAt(x, y));
            }
        }
    }
    glPointSize(3);
    ofPushMatrix();
    // the projected points are 'upside down' and 'backwards'
    ofScale(1, -1, -1);
    ofTranslate(0, 0, -500); // center the points a bit
    ofEnableDepthTest();
    mesh.drawVertices();
    ofDisableDepthTest();
    ofPopMatrix();
    
    
}


//--------------------------------------------------------------
void ofApp::exit() {
    //kinect.setCameraTiltAngle(0); // zero the tilt on exit
    kinect.close();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    switch (key) {
        case '>':
        case '.':
            farThreshold ++;
            if (farThreshold > 255) farThreshold = 255;
            break;
            
        case '<':
        case ',':
            farThreshold --;
            if (farThreshold < 0) farThreshold = 0;
            break;
            
        case '+':
        case '=':
            nearThreshold ++;
            if (nearThreshold > 255) nearThreshold = 255;
            break;
            
        case '-':
            nearThreshold --;
            if (nearThreshold < 0) nearThreshold = 0;
            break;
    }
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
