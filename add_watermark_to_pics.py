from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont
import glob


def add_text_watermark(input_img_path, output_img_path, text, text_color, rel_pos):
    image = Image.open(input_img_path)
    width, height = image.size
    abs_pos = (rel_pos[0] * width, rel_pos[1]*height)

    drawing = ImageDraw.Draw(image)
    font = ImageFont.truetype("Pillow/Tests/fonts/FreeMono.ttf", 30)
    drawing.text(abs_pos, text, fill=text_color, font=font)
    image.save(output_img_path)


if __name__ == '__main__':
    text ='@Heming Chen'
    red = (255, 0, 0)

    imgs = glob.glob("misc/*.png")
    for img in imgs:
        add_text_watermark(
            input_img_path=img,
            output_img_path=img,
            text=text,
            text_color=red,
            rel_pos=(0.3, 0.3))
