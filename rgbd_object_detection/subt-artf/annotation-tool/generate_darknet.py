"""Convert VIA Annotation Tool format to darknet annotation files"""

import json
import os
from pathlib2 import Path
import argparse
import cv2

USED_ARTF_NAME = ["phone", "helmet", "rope", "backpack", "survivor"]


def main(annotation_file, in_images_folder, out_annotations_folder):
    annotations_dir = os.path.dirname(annotation_file)

    with open(annotation_file) as f:
        json_data = json.load(f)
        train_path = os.path.join(annotations_dir, "train.txt")
        train_file = open(str(train_path), 'w')

        for item in json_data.values():
            file_name = item['filename']
            regions = item['regions']
            labels = []
            for reg in regions:
                artf_name = reg['region_attributes']['artifact']
                if artf_name not in USED_ARTF_NAME:
                    continue
                x = reg['shape_attributes']['x']
                y = reg['shape_attributes']['y']
                width = reg['shape_attributes']['width']
                height = reg['shape_attributes']['height']

                img = cv2.imread(file_name)
                img_height = img.shape[0]
                img_width = img.shape[1]

                # Store annotations in normalized darknet format
                label = dict()
                label['class_id'] = USED_ARTF_NAME.index(artf_name)
                label['x_center'] = (x + (width/2.0)) / img_width
                label['y_center'] = (y + (height/2.0)) / img_height
                label['width'] = float(width) / img_width
                label['height'] = float(height) / img_height

                labels.append( label )

                train_file.write(str(file_name)+"\n")



            darknet_label_path = get_darknet_filepath(annotations_dir, file_name)
            write_darknet_label(darknet_label_path, labels)

    
    train_file.close()
            

def get_darknet_filepath(annotations_dir, img_file_path):
    img_parent_path = os.path.dirname(img_file_path)
    img_parent_path = os.path.basename(img_parent_path)

    # assume only 1 '.' in a filename, eg: artifact_egg.jpg
    img_file_name = os.path.basename(img_file_path)
    base, _ = os.path.splitext(img_file_name)
    annotation_file_name = base + ".txt"

    relative_path = os.path.join(annotations_dir, img_parent_path, annotation_file_name)
    full_path = (Path(__file__).parent / relative_path).resolve(strict=False)
    print(full_path)

    return full_path


def write_darknet_label(filename, labels):
    # Check the path exists, else create relevant sub-folders

    try:
        Path(filename).parent.mkdir(parents=True, exist_ok=False)
    except OSError:
        pass

    darket_label_file = open(str(filename), "w")
    
    print(labels)
    for label in labels:
        out_str = "{} {} {} {} {}\n".format(label['class_id'], label['x_center'], label['y_center'], label['width'], label['height'])
        darket_label_file.write(out_str)

    darket_label_file.close()

def create_darkent_names_file(class_file_name):
    class_file = open(str(class_file_name), "w")

    for label in USED_ARTF_NAME:
        class_file.write(label + "\n")

    class_file.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Filter and convert annotations.')
    parser.add_argument('annotation', help='via json annotations file')
    parser.add_argument('--input', help='input parent images folder', default='virtual')
    parser.add_argument('--output', help='output darknet annotations folder', default='labels')
    parser.add_argument('--class-file', help='output file containing class list', default='../labels/subt_cave.names')
    args = parser.parse_args()
    annotation_file = args.annotation
    in_images_folder = args.input
    out_annotations_folder = args.output
    class_file_name = args.class_file

    create_darkent_names_file(class_file_name)
    main(annotation_file, in_images_folder, out_annotations_folder)