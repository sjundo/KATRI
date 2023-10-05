import cv2

cap = cv2.VideoCapture(0) 

# 카메라 속성 설정 (노출 조절)
exposure_value = -3  # 노출 값을 조정. 더 작은 값은 노출을 줄임
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25) 
cap.set(cv2.CAP_PROP_EXPOSURE, exposure_value)

while True:
    ret, frame = cap.read()
    
    cv2.imshow('Camera', frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
