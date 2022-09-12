#include <push_rtmp.h>
#include <opencv2/highgui.hpp>

int main(int argc, char *agrv[])
{
    VideoCapture capture;
    capture.open(2);
    if (!capture.isOpened())
    {
        cout << "Failed to open the camera!" << endl;
        return -1;
    }
    Mat frame;

    PushRTMP::Ptr rtmp;
    string rtmp_addr = "rtmp://1.116.137.21:7788/videotest";

    rtmp = make_shared<PushRTMP>(capture.get(CAP_PROP_FRAME_WIDTH),
                                 capture.get(CAP_PROP_FRAME_HEIGHT),
                                 capture.get(CAP_PROP_FPS));
    if (!rtmp->initRTMP(rtmp_addr,"BGRA"))
    {
        cout << "Failed to initialize RTMP!" << endl;
        return -1;
    }

    while (true)
    {
        capture >> frame;
        imshow("Raw Video", frame);
        if (!rtmp->startPush(frame.data, frame.elemSize()))
            cout << "Error when push!" << endl;
        waitKey(30);
    }
    return 0;
}