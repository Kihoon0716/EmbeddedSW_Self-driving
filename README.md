# EmbeddedSW_Self-driving

## 기능 및 특징

### 메뉴 구성 및 파라미터 동적 수정

변수를 담고있는 구조체와 키보드의 입력에 따라 이를 수정하는 컨트롤러로 나뉨.
컨트롤러는 생성시에 화면에 표시 될 이름, 그리고 해당 컨트롤러가 메뉴를 보여주기 위한 용도인지 파라미터 수정용인지를 나타내는 bool값을 입력받는다.
파라미터 수정을 위한 컨트롤러는 추가적으로 setParam 함수를 통해 컨트롤 할 파라미터의 수소를 저장해준다.

모든 컨트롤러를 생성한 이후 addChild 함수를 통해 트리구조의 관계설정을 해준다.
[컨트롤로 구현](https://github.com/Kihoon0716/EmbeddedSW_Self-driving/blob/master/exam_cv.cpp#L531)
[활용](https://github.com/Kihoon0716/EmbeddedSW_Self-driving/blob/master/exam_cv.cpp#L876)
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

### 동작 영상

