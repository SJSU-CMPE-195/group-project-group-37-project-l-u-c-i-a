import cv2
import pyzed.sl as sl


zed = sl.Camera()

init_params = sl.InitParameters()
init_params.camera_resolution = sl.RESOLUTION.HD720
init_params.camera_fps = 30
init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE

err = zed.open(init_params)
if err != sl.ERROR_CODE.SUCCESS:
    print("Camera open failed:", err)
    raise SystemExit(1)

object_params = sl.ObjectDetectionParameters()
object_params.enable_tracking = False
object_params.detection_model = sl.DETECTION_MODEL.MULTI_CLASS_BOX

err = zed.enable_object_detection(object_params)
if err != sl.ERROR_CODE.SUCCESS:
    print("Object detection failed:", err)
    zed.close()
    raise SystemExit(1)

objects = sl.Objects()
object_runtime = sl.ObjectDetectionRuntimeParameters()
object_runtime.detection_confidence_threshold = 40

image = sl.Mat()

try:
    while True:
        if zed.grab() == sl.ERROR_CODE.SUCCESS:
            # SDK 3.8.2 takes runtime parameters here, during retrieval.
            zed.retrieve_objects(objects, object_runtime)
            zed.retrieve_image(image, sl.VIEW.LEFT)

            frame = image.get_data()
            frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)

            if objects.is_new:
                for obj in objects.object_list:
                    bbox = obj.bounding_box_2d
                    x1 = int(bbox[0][0])
                    y1 = int(bbox[0][1])
                    x2 = int(bbox[2][0])
                    y2 = int(bbox[2][1])

                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

                    label = "{} {:.1f}%".format(obj.label, obj.confidence)
                    cv2.putText(
                        frame,
                        label,
                        (x1, max(y1 - 10, 20)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 0),
                        2,
                    )

                    print(
                        "{} | confidence: {:.1f}% | position: {}".format(
                            obj.label, obj.confidence, obj.position
                        )
                    )

            cv2.imshow("ZED 2 Object Detection", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
finally:
    zed.disable_object_detection()
    zed.close()
    cv2.destroyAllWindows()