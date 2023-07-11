import cv2
import numpy as np

# 신호등 클래스와 해당 클래스에 맞는 색상 구간을 정의합니다.
class_colors = {
    'green': {'lower': (45, 50, 50), 'upper': (75, 255, 255)},
    'yellow': {'lower': (20, 50, 50), 'upper': (40, 255, 255)},
    'red': {'lower': (0, 50, 50), 'upper': (10, 255, 255)}
}

# 검출된 객체의 색상 정보가 정해진 구간에 맞는지 확인하는 함수입니다.
def check_color(hsv_color, class_name):
    lower = class_colors[class_name]['lower']
    upper = class_colors[class_name]['upper']
    return cv2.inRange(hsv_color, lower, upper)

# YOLOv5로 학습된 모델을 로드합니다.
model = cv2.dnn.readNet('best.pt')

# 카메라로부터 영상을 읽어오는 객체를 생성합니다.
cap = cv2.VideoCapture(0)

while True:
    # 영상을 프레임 단위로 읽어옵니다.
    ret, frame = cap.read()

    # 이미지를 전처리합니다.
    blob = cv2.dnn.blobFromImage(frame, scalefactor=1/255.0, size=(640, 640), swapRB=True)

    # 전처리된 이미지를 모델에 입력하여 객체를 검출합니다.
    model.setInput(blob)
    outputs = model.forward()

    # 검출된 객체들을 처리합니다.
    for output in outputs:
        for detection in output:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]

            # 신뢰도가 일정 값 이상인 객체를 검출합니다.
            if confidence > 0.5:
                # 객체의 위치와 크기를 추출합니다.
                center_x = int(detection[0] * frame.shape[1])
                center_y = int(detection[1] * frame.shape[0])
                width = int(detection[2] * frame.shape[1])
                height = int(detection[3] * frame.shape[0])

                # 객체를 사각형으로 표시합니다.
                cv2.rectangle(frame, (center_x - width // 2, center_y - height // 2),
                              (center_x + width // 2, center_y + height // 2), (0, 255, 0), 2)

                # 객체의 색상 정보를 추출합니다.
                roi = frame[center_y - height // 2:center_y + height // 2,
                            center_x - width // 2:center_x + width // 2]
                hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

                # 색상 정보가 정해진 구간에 맞는지 확인합니다.
                color_match = check_color(hsv_roi.mean(axis=(0, 1)), class_names[class_id])
                if color_match:
                    print(f"옳게 검출한 {class_names[class_id]} 객체입니다.")

    # 결과를 출력합니다.
    cv2.imshow('Frame', frame)

    # 'q' 키를 누르면 종료합니다.
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 사용이 끝나면 객체를 해제합니다.
cap.release()
cv2.destroyAllWindows()

