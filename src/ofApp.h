#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxCvHaarFinder.h"
#include "ofxKinect.h"

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
    
    ofxKinect kinect;
    
    ofxCvGrayscaleImage	grayImage;
    ofxCvGrayscaleImage	depthImage;
    ofxCvColorImage		colorImage;
    ofxCvGrayscaleImage	redImage;
    
    ofxCvContourFinder 	contourFinder;
    
    ofImage img;
    ofxCvHaarFinder finder;
    
    int nearThreshold;
    int farThreshold;
    
    bool bDrawPointCloud;
    
    int angle;
    
    // used for viewing the point cloud
    ofEasyCam easyCam;
    ofCamera cam;
    
    vector<ofMesh> totalMesh;
    int lastSecond = 0;
    int w;
    int h;
    int minConnectionDistance; //how far can we connect vertexes
    int maxConnectionDistance; //how far can we connect vertexes
    int nearby;//how far to sample the neighbor
    
    int minFigureDistance; //min distance from camera things being recorded should be
    int maxFigureDistance; //min distance from camera things being recorded should be
    
    int lastSampleLoc[2];
    
    ofQuaternion qFromV(ofVec3f u, ofVec3f v);
    ofQuaternion twoToQ(ofxCvBlob blob0, ofxCvBlob blob1);
    float blobAngle;
    ofVec3f blobAxis;
    ofQuaternion blobQ;
};
