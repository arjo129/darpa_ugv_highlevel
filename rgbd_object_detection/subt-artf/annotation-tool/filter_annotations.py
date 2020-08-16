"""Filter and convert annotations"""

import json
import os
import cv2

USED_ARTF_NAME = ["phone", "helmet", "rope", "backpack", "survivor", "robot"]


def main(annotation_file, out_prefix):
    annotations_dir = os.path.dirname(annotation_file)
    data = []
    with open(annotation_file) as f:
        json_data = json.load(f)
        for item in json_data.values():
            file_name = item['filename']
            regions = item['regions']
            for reg in regions:
                artf_name = reg['region_attributes']['artifact']
                if artf_name not in USED_ARTF_NAME:
                    continue
                x = reg['shape_attributes']['x']
                y = reg['shape_attributes']['y']
                width = reg['shape_attributes']['width']
                height = reg['shape_attributes']['height']
                # Store annotations and add a label about a future use. "None" in the beginning (do not use).
                data.append( [file_name, artf_name, [x, y, x + width, y+ height], "None"] )
    ii = 0
    while True:
        if ii < 0:
            ii = 0
        if ii >= len(data):
            ii = len(data) -1
        file_name, artf_name, bbox, use_for = data[ii]
        x, y, xw, yh = bbox
        img = cv2.imread(os.path.join(annotations_dir, file_name), 1)
        cv2.rectangle(img, (x,y), (xw, yh), (0,0,255))
        cv2.putText(img, artf_name, (x,y),fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(0,0,255))
        cv2.putText(img, use_for, (10, 50), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(255, 0, 0))
        cv2.imshow("win", img)

        k = cv2.waitKey(0) & 0xFF
        if k == ord("n"): # next img
            ii += 1
        elif k == ord("b"):  # back one img
            ii -= 1
        elif k == ord("t"):
            data[ii][3] = "train" # use for training
        elif k == ord("e"):
            data[ii][3] = "eval" # use for evaluation
        elif k == ord("d"):
            data[ii][3] = "None" # do not use
        elif k == ord("q"):  # close and save
            break

    cv2.destroyAllWindows()

    out_train = open(out_prefix + "_train.csv", "w")
    out_eval = open(out_prefix + "_eval.csv", "w")
    out_train.write("filename,class,xmin,ymin,xmax,ymax\r\n")
    out_eval.write("filename,class,xmin,ymin,xmax,ymax\r\n")
    for item in data:
        file_name, artf_name, bbox, use_for = item
        x, y, xw, yh = bbox
        output_string = "%s,%s,%d,%d,%d,%d\r\n" %(file_name, artf_name, x, y, xw, yh)
        if use_for == "train":
            out_train.write(output_string)
        elif use_for == "eval":
            out_eval.write(output_string)

    out_train.close()
    out_eval.close()


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description='Filter and convert annotations.')
    parser.add_argument('annotation', help='json - annotations')
    parser.add_argument('--out', help='outpput csv filename prefix', default='annotation')
    args = parser.parse_args()
    annotation_file = args.annotation
    out_csv = args.out
    main(annotation_file, out_csv)
