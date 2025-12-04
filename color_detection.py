import cv2
import matplotlib.pyplot as plt
import numpy as np
import os


def color_detection(image):
    """
    Detects edges around a specific neon-pink color (RGB 221,101,121)
    and shows debug plots at each stage.
    """

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    bgr_color = np.uint8([[[121, 101, 221]]])
    hsv_color = cv2.cvtColor(bgr_color, cv2.COLOR_BGR2HSV)[0][0]

    h, s, v = hsv_color
    lower_bound = np.array([max(h - 5, 0), max(s - 50, 0), max(v - 50, 0)])
    upper_bound = np.array([min(h + 5, 179), min(s + 50, 255), min(v + 50, 255)])

    print(f"HSV bounds: {lower_bound} - {upper_bound}")

    mask = cv2.inRange(hsv, lower_bound, upper_bound)
    # plt.imshow(mask, cmap="gray")
    # plt.title("Neon pink mask")
    # plt.axis("off")
    # plt.show()

    edges = cv2.Canny(mask, 50, 150)
    # plt.imshow(edges, cmap="gray")
    # plt.title("Canny edges")
    # plt.axis("off")
    # plt.show()

    edges_closed = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    # plt.imshow(edges_closed, cmap="gray")
    # plt.title("Edges (closed)")
    # plt.axis("off")
    # plt.show()

    contours, _ = cv2.findContours(
        edges_closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    debug = image.copy()
    MIN_AREA = 50
    contours = [c for c in contours if cv2.contourArea(c) >= MIN_AREA]

    if len(contours) > 0:
        xs, ys, xe, ye = [], [], [], []
        for cnt in contours:
            x, y, w, h = cv2.boundingRect(cnt)
            xs.append(x)
            ys.append(y)
            xe.append(x + w)
            ye.append(y + h)

        X1, Y1 = min(xs), min(ys)
        X2, Y2 = max(xe), max(ye)
        merged_box = (X1, Y1, X2 - X1, Y2 - Y1)
        cv2.rectangle(debug, (X1, Y1), (X2, Y2), (255, 0, 0), 3)
    else:
        merged_box = None

    plt.imshow(cv2.cvtColor(debug, cv2.COLOR_BGR2RGB))
    plt.title("Merged bounding rectangle")
    plt.axis("off")
    plt.show()
    return merged_box


# image = cv2.imread(f"color-training-images/image0.png")
# contours, mask, edges_closed = screen_detection(image)


for i in range(0, 50):
    image = cv2.imread(f"color-training-images/image{i}.png")
    contours, mask, edges_closed = color_detection(image)
