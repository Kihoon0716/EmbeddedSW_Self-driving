# EmbeddedSW_Self-driving

## 기능 및 특징

### 파라미터 동적 수정


```cpp
    p = params;
    //main
    Controller * main = new Controller("Main", true);
    
        // mian -> line detection
        Controller * lineDetection = new Controller("Line Detection", true);
            // mian -> line detection -> homography
            Controller * homography = new Controller("Homography", true);
				Controller * upperPoint = new Controller("Upper Point", false);// 파라미터
				upperPoint->setParam(&params->lineDetection.homography.upperPoint);
            // mian -> line detection -> binarization
            Controller * binarization = new Controller("Binarization", false);
            binarization->setParam(&params->lineDetection.binarization);

    
        // main -> traffic light
        Controller * trafficLight = new Controller("Traffic Light", true);
            //main -> traffic light -> traffic light red
            Controller * trafficLightRed = new Controller("traffic Light Red", false);
            trafficLightRed->setParam(&params->trafficLight.red);
            Controller * trafficLightGreen = new Controller("traffic Light Green", false);
            trafficLightGreen->setParam(&params->trafficLight.green);
            Controller * trafficLightorange = new Controller("traffic Light Orange", false);
            trafficLightorange->setParam(&params->trafficLight.orange);
            Controller * trafficLightArrow = new Controller("traffic Light Arrow", false);
            trafficLightArrow->setParam(&params->trafficLight.arrow);
            Controller * trafficLightMaxSize = new Controller("traffic Light Max Size", false);
            trafficLightMaxSize->setParam(&params->trafficLight.sizeMax);
            Controller * trafficLightMinSize = new Controller("traffic Light Min Size", false);
            trafficLightMinSize->setParam(&params->trafficLight.sizeMin);
            Controller * trafficLightCircleConvexityMin = new Controller("traffic Light Circle Convexity Min", false);
            trafficLightCircleConvexityMin->setParam(&params->trafficLight.circleConvexityMin);    
            Controller * trafficLightArrowConvexityMax = new Controller("traffic Light Arrow Convexity Max", false);
            trafficLightArrowConvexityMax->setParam(&params->trafficLight.arrowConvexityMax);
            Controller * trafficLightCircleCircularityMin = new Controller("traffic Light Circle Circularity Min", false);
            trafficLightCircleCircularityMin->setParam(&params->trafficLight.circleCircularityMin);    
            Controller * trafficLightArrowCircularityMax = new Controller("traffic Light arrow Circularity Max", false);
            trafficLightArrowCircularityMax->setParam(&params->trafficLight.arrowCircularityMax);
        // main -> stop sign
        Controller * stopSign = new Controller("StopSign", true);
            // main -> stop sign -> Hsv
            Controller * stopSignHsv = new Controller("Stop Sign Hsv", false);
            stopSignHsv->setParam(&params->stopsign.red);
            Controller * stopSignMaxSize = new Controller("Stop Sign Size Max", false);
            stopSignMaxSize->setParam(&params->stopsign.sizeMax);
            Controller * stopSignMinSize = new Controller("Stop Sign Size Min", false);
            stopSignMinSize->setParam(&params->stopsign.sizeMin);
            Controller * stopSignMaxConvexity = new Controller("Stop Sign Max Convexity", false);
            stopSignMaxConvexity->setParam(&params->stopsign.convexityMax);
            Controller * stopSignMinConvexity = new Controller("Stop Sign Min Convexity", false);
            stopSignMinConvexity->setParam(&params->stopsign.convexityMin);
            Controller * stopSignMaxCircularity = new Controller("Stop Sign Max Circularity", false);
            stopSignMaxCircularity->setParam(&params->stopsign.circularityMax);
            Controller * stopSignMinCircularity = new Controller("Stop Sign Min Circularity", false);
            stopSignMinCircularity->setParam(&params->stopsign.circularityMin);
        
        // main-> mode
        Controller * mode = new Controller("Mode", true);
            //main -> mode -> cameraAngle
            Controller * cameraAngle = new Controller("Camera Angle", false);
            cameraAngle->setParam(&params->mode.cameraAngle);
            Controller * minPixelCount = new Controller("Min Pixel Count", false);
            minPixelCount->setParam(&params->mode.minPixelCount);
            Controller * maxPixelVariance = new Controller("Max Pixel Variance", false);
            maxPixelVariance->setParam(&params->mode.maxVariance);
    main->addChild(lineDetection)
        ->addChild(trafficLight)
        ->addChild(stopSign)
        ->addChild(mode)
        ->setIsMain();
		lineDetection->addChild(homography)
			->addChild(binarization);
		    homography->addChild(upperPoint);
        trafficLight->addChild(trafficLightRed)
            ->addChild(trafficLightGreen)
            ->addChild(trafficLightorange)
            ->addChild(trafficLightArrow)
            ->addChild(trafficLightMaxSize)
            ->addChild(trafficLightMinSize)
            ->addChild(trafficLightCircleCircularityMin)
            ->addChild(trafficLightArrowCircularityMax)
            ->addChild(trafficLightCircleConvexityMin)
            ->addChild(trafficLightArrowConvexityMax);
        stopSign->addChild(stopSignHsv)
            ->addChild(stopSignMaxSize)
            ->addChild(stopSignMinSize)
            ->addChild(stopSignMaxConvexity)
            ->addChild(stopSignMinConvexity)
            ->addChild(stopSignMaxCircularity)
            ->addChild(stopSignMinCircularity);
        mode->addChild(cameraAngle)
            ->addChild(minPixelCount)
            ->addChild(maxPixelVariance);
    main->start();
```
### 화면전환

### 소켓통신으로 영상 수신

### 시점변환

### 로지컬 무빙

## 동작 영상

