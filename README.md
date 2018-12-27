# EmbeddedSW_Self-driving

## 기능 및 특징

### 파라미터 동적 수정


```cpp
    p = params; // 파라미터 객체

  // 각 메뉴에 해당하는 컨트롤러 생성
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

.....

  // 각 메뉴의 자식, 부모관계 설정
    main->addChild(lineDetection)
        ->addChild(trafficLight)
        ->addChild(stopSign)
        ->addChild(mode)
        ->setIsMain();
.....
  // 키보드 컨트롤러 실행
    main->start();
```
### 화면전환

### 소켓통신으로 영상 수신

### 시점변환

### 로지컬 무빙

## 동작 영상

