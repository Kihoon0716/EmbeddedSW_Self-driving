# EmbeddedSW_Self-driving

## 기능 및 특징

### 메뉴 구성 및 파라미터 동적 수정

변수를 담고있는 구조체와 키보드의 입력에 따라 이를 수정하는 컨트롤러로 나뉨.
컨트롤러는 생성시에 화면에 표시 될 이름, 그리고 해당 컨트롤러가 메뉴를 보여주기 위한 용도인지 파라미터 수정용인지를 나타내는 bool값을 입력받는다.
파라미터 수정을 위한 컨트롤러는 추가적으로 setParam 함수를 통해 컨트롤 할 파라미터의 수소를 저장해준다.

모든 컨트롤러를 생성한 이후 addChild 함수를 통해 트리구조의 관계설정을 해준다.

[컨트롤로 구현 소스코드](https://github.com/Kihoon0716/EmbeddedSW_Self-driving/blob/master/exam_cv.cpp#L531)

[활용 소스코드](https://github.com/Kihoon0716/EmbeddedSW_Self-driving/blob/master/exam_cv.cpp#L876)

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
### 화면전환 및 소켓통신으로 영상 송수신

영상처리의 각 기능별로 모니터링이 가능하게끔 화면전환 기능을 추가하였다.
```cpp
 				else if(key == "1"){  // 원본 화면
                    p->mode.view = 1;
                }else if(key == "2"){  // homography 상단 점
                    p->mode.view = 2;
                }else if(key == "3"){  // homography rgb
                    p->mode.view = 3;
                }else if(key == "4"){  // homography 이진화
                    p->mode.view = 4;
                }else if(key == "5"){  // 돌발표지
                    p->mode.view = 5;
                }else if(key == "6"){  // 신호등 빨간색
                    p->mode.view = 6;
                }else if(key == "7"){  // 신호등 주황색
                    p->mode.view = 7;
                }else if(key == "8"){  // 신호등 초록색
                    p->mode.view = 8;
                }else if(key == "9"){  // 신호등 화살표
                    p->mode.view = 9;
                }
```
[영상송신 소스코드](https://github.com/Kihoon0716/EmbeddedSW_Self-driving/blob/master/exam_cv.cpp#L123)

[영상수신 소스코드](https://github.com/Kihoon0716/EmbeddedSW_Self-driving/blob/master/server.cpp#L1)


### 로지컬 무빙

추차장미션, 추월차로 미션과 같이 하드코딩이 필요한 미션을 위해서 정해진 로직대로 움직이는것을 쉽고 빠르게 구현이 가능하도록 LogicalMove라는 클래스를 만들었다.
해당 클래스는 addLogic이라는 함수를 활용해서 로직을 여러 개 추가해서 시나리오대로 움직이도록 하는것이 가능하다.
아래와 같이 하나의 로직은 인풋값으로 바퀴의 각도, 이동거리, 벽을 감지했을 때 멈출것인지, 이동이 완료된 후에 부저를 울릴것인지 등의 값을 입력받는다.
```cpp
logic(int angle, int distance, bool detectWall, bool buzzer)
```
[소스코드](https://github.com/Kihoon0716/EmbeddedSW_Self-driving/blob/master/exam_cv.cpp#L313)

## 동작 영상
[주차장 + 차단바 테스트](https://www.youtube.com/watch?v=lu_Kf-L-fDY)

[추월차로 + 신호등](https://www.youtube.com/watch?v=-OvMKWkKgOg)