def calibrated(frame):

    L= para_distort()

    mtx = L[0]
    dist = L[1]
    # print(mtx)
    # print(dist)

    cap = cv.VideoCapture(2)

    if not cap.isOpened():
        print("Cannot open camera")
        exit()
    while True:
        # for fname in images:
        ret, frame = cap.read()
        # if frame is read correctly ret is True
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        rows, cols = gray.shape
        M = cv.getRotationMatrix2D(((cols - 1) / 2.0, (rows - 1) / 2.0), 180, 1)
        dst = cv.warpAffine(gray, M, (cols, rows))
        h, w = frame.shape[:2]
        newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))

        # undistort
        dst = cv.undistort(frame, mtx, dist, None, newcameramtx)
        # crop the image
        x, y, w, h = roi
        dst = dst[y:y + h, x:x + w]
        cv.imshow('la', dst)
        #cv.imshow('img', frame)
        if cv.waitKey(1) == ord('q'):
            break

    cap.release()
    cv.destroyAllWindows()


