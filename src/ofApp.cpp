#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    
    //---EDITABLES
    //OPENCV Threshold
    nearThreshold = 0;
    farThreshold  = 150;
    minBlobSize = 900;
    maxBlobSize = (kinect.width*kinect.height)/1;
    
    //MyPointCloud Threshold
    w = 640;
    h = 480;
    nearby = 20;
    minConnectionDistance = 50;
    maxConnectionDistance = 80;
    minFigureDistance = 400;
    maxFigureDistance = 4000;
    
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
    
    kinect.setCameraTiltAngle(-22);
    
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
    
    depthImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);
    depthImage.flagImageChanged();
    
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
        
        //make sure left blob always on left
        if(blob1.centroid[0] < blob0.centroid[0]){
            ofxCvBlob temp = blob1;
            blob1 = blob0;
            blob0 = temp;
        }
        
        /*
         finder.findHaarObjects(redImage);
         if(finder.blobs.size() >= 2){
         
         ofxCvBlob blob0 = finder.blobs[0];
         ofxCvBlob blob1 = finder.blobs[1];
         */
        
        double deg = getAngleBetweenBlobs(blob0, blob1);
        
        if(deg != -1){
            //ofLogNotice() << deg;
            
            easyCam.orbit(deg, 0,500);

        }
        
    }
    
    
}

//--------------------------------------------------------------
void ofApp::draw(){
    ofBackground(0,0,0);
    ofSetColor(255, 255, 255);
    
    /** DEBUG **/
    
    depthImage.draw(15, 15, 400, 300);
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
    
    ofSetColor(0,0,255);
    ofNoFill();
    ofDrawBox(0,0, 0, 50, 50, 100);
    
    drawPointCloud();
    //drawPointCloudAll();
    
    ofPopMatrix();
    
    easyCam.end();
    
    
}

double ofApp::getAngleBetweenBlobs(ofxCvBlob blob0, ofxCvBlob blob1){
    
    ofVec3f blobWorldPos0 =  kinect.getWorldCoordinateAt(blob0.centroid[0], blob0.centroid[1]);
    ofVec3f blobWorldPos1 =  kinect.getWorldCoordinateAt(blob1.centroid[0], blob1.centroid[1]);

    
    double arc = atan2(blobWorldPos1[2] - blobWorldPos0[2], blobWorldPos1[0] - blobWorldPos0[0]);
    double degrees = arc  * 180 / PI;
    
    //if (degrees < 0) degrees += 360;
    //else if (degrees > 360) degrees -= 360;
    
    if(blobWorldPos0[0] != 0 && blobWorldPos0[2] != 0 && blobWorldPos1[0] != 0  && blobWorldPos1[2] != 0) {
        ofLogNotice() << blob0.centroid[0] << " " << blob0.centroid[1];
        ofLogNotice() << "A BLOB: " << blobWorldPos0[0] << ", " << blobWorldPos0[2];
        ofLogNotice() << "B BLOB: " << blobWorldPos1[0] << ", " << blobWorldPos1[2];
        ofLogNotice() << "DIFF: " << blobWorldPos1[2] - blobWorldPos0[2] << ", " << blobWorldPos1[0] - blobWorldPos0[0];
        ofLogNotice() << "FINAL: " << degrees;
        
        return degrees;
    }
    else
        return -1;
    
}


void ofApp::drawPointCloud() {
    ofMesh *currentMesh = &totalMesh[totalMesh.size()-1];
    
    currentMesh->setMode(OF_PRIMITIVE_POINTS);
    
    
    
    for(int i = 0; i < 2; i++){
        
        float* x = &lastSampleLoc[i][0];
        float* y = &lastSampleLoc[i][1];
        
        double multiplier = 1;
        if(contourFinder.nBlobs >= 2){
            int other = i == 0 ? 1 : 0;
            multiplier = (contourFinder.blobs.at(i).area/contourFinder.blobs.at(other).area);
            //ofLogNotice() << i << " " << other << " " <<multiplier;
        }
        
        
        if(kinect.getDistanceAt(*x,*y) > minFigureDistance && kinect.getDistanceAt(*x,*y) < maxFigureDistance) {
            
            ofColor c = kinect.getColorAt((int)*x,(int)*y);
            
            if(multiplier > 2 || multiplier < .5)
            c.setBrightness(255 * multiplier);
            
            if(contourFinder.nBlobs == 1)
            c.setSaturation( 128 );
            
            currentMesh->addColor(c);
            currentMesh->addVertex(kinect.getWorldCoordinateAt((int)*x, (int)*y));
        }
        
        *x += ofRandom((-1*nearby), nearby);
        *y += ofRandom((-1*nearby), nearby);
        
    }
    
    //draw each point every frame
    ofPushMatrix();
    ofScale(-1,-1,-1);
    ofTranslate(0, -100, -1000);
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
        
        //Set new sample loc
        for(int i = 0; i < contourFinder.nBlobs; i++){
            ofxCvBlob blob = contourFinder.blobs.at(i);
            ofPoint thePoint = blob.pts[ofRandom(0, blob.nPts)];
            
            lastSampleLoc[i] = thePoint;
            //we have to reflect on Y axis since Kinect is going to be drawn backward
            lastSampleLoc[i][0] = ofMap(lastSampleLoc[i][0], 0, w, w, 0);
            
        }
        
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
    ofScale(-1, -1, -1);
    ofTranslate(0, 0, -800); // center the points a bit
    ofEnableDepthTest();
    mesh.drawVertices();
    ofDisableDepthTest();
    ofPopMatrix();
    
    for(int i = 0; i < contourFinder.nBlobs; i++){
        ofxCvBlob blob = contourFinder.blobs.at(i);
        
        for(int j = 0; j<blob.nPts; j++){
            ofCircle(blob.pts[j][0],blob.pts[j][1],5);
        }
    }
    
    
    
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
