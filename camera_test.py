import cv2

cap = cv2.VideoCapture(0)

while True:
    frame = cap.read()[1]  # returns (error code, frame), so get 2nd element
    hsvframe = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    bluelbound = (100, 50, 50)
    blueubound = (130, 255, 255)
    yellowlbound = (30, 50, 50)
    yellowubound = (50, 255, 255)

    bluemask = cv2.inRange(hsvframe, bluelbound, blueubound)
    yellowmask = cv2.inRange(hsvframe, yellowlbound, yellowubound)

    bluearea = cv2.countNonZero(bluemask)
    yellowarea = cv2.countNonZero(yellowmask)

    dominantcolor = "None"
    if bluearea > yellowarea and bluearea > 0:
        dominantcolor = "Blue"
    elif yellowarea > bluearea and yellowarea > 0:
        dominantcolor = "Yellow"

    bluecontours, _ = cv2.findContours(bluemask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in bluecontours:
        area = cv2.contourArea(contour)
        if area > 200:
            x, y, w, h = cv2.boundingRect(contour)
            cv2.drawContours(frame, [contour], -1, (255, 0, 0), 1)

    biggest_contour = max(bluecontours, key=cv2.contourArea) if bluecontours else None
    if biggest_contour is not None and dominantcolor == "Blue":
        x, y, w, h = cv2.boundingRect(biggest_contour)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)

    yellowcontours, _ = cv2.findContours(yellowmask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in yellowcontours:
        area = cv2.contourArea(contour)
        if area > 25:
            x, y, w, h = cv2.boundingRect(contour)
            cv2.drawContours(frame, [contour], -1, (0, 255, 255), 1)

    biggest_contour = max(yellowcontours, key=cv2.contourArea) if yellowcontours else None
    if biggest_contour is not None and dominantcolor == "Yellow":
        x, y, w, h = cv2.boundingRect(biggest_contour)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 2)

    cv2.imshow("bluemask", bluemask)
    cv2.imshow("yellowmask", yellowmask)
    cv2.imshow("frame", frame)
    cv2.waitKey(1)