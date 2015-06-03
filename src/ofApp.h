#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxPostProcessing.h"

class ofApp : public ofBaseApp{
    
public:
    void setup();
    void update();
    void draw();
    void exit();
    
    void drawPointCloud();
    void drawPointCloudAll();
    
    void keyPressed(int key);
    void keyReleased(int key);
    void mouseMoved(int x, int y );
    void mouseDragged(int x, int y, int button);
    void mousePressed(int x, int y, int button);
    void mouseReleased(int x, int y, int button);
    void windowResized(int w, int h);
    void dragEvent(ofDragInfo dragInfo);
    void gotMessage(ofMessage msg);
    
private:
    void numBlobsIndicator(int n);
    void startTotalMesh();
    int timeSegment = 8;
    int nextSegmentDistance = -500;
    int startCameraDist = 250;
    int cameraDist = startCameraDist;
    bool controlBlobsManually = false;
    int numBlobs = 0;
    int usedBlobs =-1;
    ofxKinect kinect;
    
    ofxCvGrayscaleImage	grayImage;
    ofxCvGrayscaleImage	depthImage;
    ofxCvColorImage		colorImage;
    ofxCvGrayscaleImage	redImage;
    
    ofxCvContourFinder 	contourFinder;
    
    int nearThreshold;
    int farThreshold;;
    
    // used for viewing the point cloud
    ofEasyCam easyCam;
    
    vector<vector<ofMesh> > totalMesh;
    int lastMinute = 0;
    int lastSecond = 0;
    
    int w;
    int h;
    int minConnectionDistance; //how far can we connect vertexes
    int maxConnectionDistance; //how far can we connect vertexes
    int nearby;//how far to sample the neighbor
    
    int minFigureDistance; //min distance from camera things being recorded should be
    int maxFigureDistance; //min distance from camera things being recorded should be
    
    ofPoint lastSampleLoc[2];
    double getAngleBetweenBlobs(ofxCvBlob blob0, ofxCvBlob blob1);
    ofQuaternion qFromV(ofVec3f u, ofVec3f v);
    ofQuaternion twoToQ(ofxCvBlob blob0, ofxCvBlob blob1);
    
    double minBlobSize;
    double maxBlobSize;
    
};
