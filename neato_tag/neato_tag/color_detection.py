import cv2
import matplotlib.pyplot as plt
import numpy as np


def color_detection(image, neato=None):
    """
    Detects bounding box of a specified color in an image and returns it
    """
    MIN_AREA = 20
    TOLERANCE = 20

    # If the neato is neato2, use its specific color, otherwise use neato1's
    if neato == "neato2":
        sample_rgb = np.array([187, 60, 89])
    else:
        sample_rgb = np.array([141, 55, 77])

    # Compute lower and upper bounds
    lower_bound = np.maximum(sample_rgb - TOLERANCE, 0)
    upper_bound = np.minimum(sample_rgb + TOLERANCE, 255)

    # Convert image to RGB
    rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    plt.imshow(rgb_image, cmap="gray")
    plt.title("Neon pink mask")
    plt.axis("off")
    plt.show()

    # Create the mask
    mask = cv2.inRange(rgb_image, lower_bound, upper_bound)
    plt.imshow(mask, cmap="gray")
    plt.title("Neon pink mask")
    plt.axis("off")
    plt.show()

    # Detect edges
    edges = cv2.Canny(mask, 50, 150)
    plt.imshow(edges, cmap="gray")
    plt.title("Canny edges")
    plt.axis("off")
    plt.show()

    # Close any edges
    edges_closed = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    plt.imshow(edges_closed, cmap="gray")
    plt.title("Edges (closed)")
    plt.axis("off")
    plt.show()

    # Find contours
    contours, _ = cv2.findContours(
        edges_closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    debug = image.copy()

    # Keep contours larger than the minimum area
    contours = [c for c in contours if cv2.contourArea(c) >= MIN_AREA]
    cv2.drawContours(
        debug,
        contours,
        contourIdx=-1,  # draw all contours
        color=(0, 255, 255),  # yellow for visibility
        thickness=2,
    )
    plt.imshow(cv2.cvtColor(debug, cv2.COLOR_BGR2RGB))
    plt.title("Merged bounding rectangle")
    plt.axis("off")
    plt.show()
    debug = image.copy()
    # If there are still contours remaining, merge them together to form
    # a larger rectangle
    if len(contours) > 0:
        xs, ys, xe, ye = [], [], [], []
        for cnt in contours:
            x, y, w, h = cv2.boundingRect(cnt)
            # contour_crop = image[y : y + h, x : x + w].copy()
            xs.append(x)
            ys.append(y)
            xe.append(x + w)
            ye.append(y + h)
            # plt.imshow(cv2.cvtColor(contour_crop, cv2.COLOR_BGR2RGB))
            # plt.axis("off")
            # plt.show()

        X1, Y1 = min(xs), min(ys)
        X2, Y2 = max(xe), max(ye)
        merged_box = [X1, Y1, X2, Y2]
        cv2.rectangle(debug, (X1, Y1), (X2, Y2), (255, 0, 0), 3)
        # plt.imshow(cv2.cvtColor(debug, cv2.COLOR_BGR2RGB))
        # plt.title("Merged bounding rectangle")
        # plt.axis("off")
        # plt.show()
    # If no contours remain, return set mergged_box to None
    else:
        merged_box = None

    plt.imshow(cv2.cvtColor(debug, cv2.COLOR_BGR2RGB))
    plt.title("Merged bounding rectangle")
    plt.axis("off")
    plt.show()
    return merged_box


def convert_to_heading_angle(bounding_box, img_width, hfov=120):
    """
    Convert and get angle of center of bounding box
    """
    if bounding_box is None:
        return None

    left_x = bounding_box[0]
    right_x = bounding_box[2]

    # Map pixels to angles relative to center
    left_angle = ((left_x - img_width / 2) / (img_width / 2)) * (hfov / 2)
    right_angle = ((right_x - img_width / 2) / (img_width / 2)) * (hfov / 2)
    center_angle = (left_angle + right_angle) / 2
    return center_angle


# Functions below are for debugging


def draw_angle_markers(image, bounding_box, hfov=120):
    """
    Show the angle the code thinks is the angle of the center of bounding box
    and draw on top of the bounding box for visual debugging
    """
    if bounding_box is None:
        return image

    img_height, img_width = image.shape[:2]

    # get center angle
    center_angle = convert_to_heading_angle(bounding_box, img_width, hfov)
    if center_angle is None:
        return image

    left_x, top_y, right_x, bottom_y = bounding_box
    center_x = (left_x + right_x) // 2

    debug = image.copy()

    cv2.rectangle(debug, (left_x, top_y), (right_x, bottom_y), (0, 255, 0), 2)

    cv2.line(
        debug,
        (center_x, 0),
        (center_x, img_height),
        (0, 255, 0),  # green line
        2,
    )

    angle_text = f"{center_angle:.1f}"
    text_x = max(center_x - 40, 0)
    text_y = max(top_y - 10, 30)

    cv2.putText(
        debug,
        angle_text,
        (text_x, text_y),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 255, 0),
        2,
    )

    return debug


def check_color_detection(image_file_path):
    """
    Function to run color detection and angle calculation on specified image
    """
    image = cv2.imread(image_file_path)
    bbox = color_detection(image)

    print(bbox)

    marked_image = draw_angle_markers(image, bbox)

    plt.imshow(cv2.cvtColor(marked_image, cv2.COLOR_BGR2RGB))
    plt.title("Bounding box with angle markers")
    plt.axis("off")
    plt.show()
