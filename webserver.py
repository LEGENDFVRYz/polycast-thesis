import os
from flask import Flask, Response, render_template, redirect, url_for
from werkzeug.utils import secure_filename
from image_processing import generate_frames
from config import BROWSER_WS_PORT

app = Flask(__name__, template_folder="templates")
GALLERY_PATH = os.path.join(app.static_folder, 'gallery_images')


@app.route("/")
def index():
    is_setup = True   # if prototype is setup properly
    return render_template("index.html", is_setup=is_setup)

@app.route("/admin")
def admin_page():
    return render_template("admin.html", ws_port=BROWSER_WS_PORT)

@app.route("/client")
def client_page():
    return render_template("client.html", ws_port=BROWSER_WS_PORT)


@app.route("/gallery")
def gallery_page():
    is_setup = True
    
    folders = []
    try:
        # Check the contebt of the gallery - fetch only the folder
        with os.scandir(GALLERY_PATH) as entries:
            folders = [entry.name for entry in entries if entry.is_dir()]
    except FileNotFoundError:
        print("No Gallery Folder Yet")
        pass

    return render_template(
        "gallery.html", 
        is_setup=is_setup, 
        ws_port=BROWSER_WS_PORT,
        folders=folders  # <-- Pass the list of folders to the template
    )


@app.route("/gallery/<string:foldername>")
def gallery_folder(foldername):
    image_extensions = {'.jpg', '.png'}
    foldername = secure_filename(foldername)
    
    folder_path = os.path.join(GALLERY_PATH, foldername)

    # Verify the folder exists and is a directory
    if not os.path.isdir(folder_path):
        print("FOLDER: Does not exist")
        return redirect(url_for("gallery_page"))

    try:
        # Fetch all the valid images in the folder
        with os.scandir(folder_path) as folder:
            images = sorted(
                (item.name for item in folder
                 if item.is_file() and os.path.splitext(item.name)[1].lower() in image_extensions),
                key=str.lower
            )
    except (FileNotFoundError, PermissionError):
        print("FOLDER: Access Error")
        return redirect(url_for("gallery_page"))


    return render_template(
        "gallery_folderview.html",
        foldername=foldername,
        images=images
    )


@app.route("/stream")
def stream_page():
    return render_template("stream.html", ws_port=BROWSER_WS_PORT)

@app.route("/video_feed")
def video_feed():
    return Response(generate_frames(), mimetype="multipart/x-mixed-replace; boundary=frame")

def run_webserver(port):
    print(f"[FLASK] running on 0.0.0.0:{port}")
    app.run(
        host="0.0.0.0",
        port=port,
        debug=True,   # ✅ enables development mode
        threaded=True
    )

if __name__ == "__main__":
    run_webserver(5000)
