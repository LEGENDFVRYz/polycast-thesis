import os
import time  # <-- ADDED: For simulating delays
import json  # <-- ADDED: For sending SSE data
from flask import Flask, Response, render_template, redirect, url_for, request, session
from auth import init_auth_db, register_admin, verify_admin
from werkzeug.utils import secure_filename
from image_processing import generate_frames
from config import BROWSER_WS_PORT

app = Flask(__name__, template_folder="templates")
app.secret_key = "polycast-creator_BatsiKuruSyaniOmit"

GALLERY_PATH = os.path.join(app.static_folder, 'gallery_images')
init_auth_db()

# --- NEW: Global variable to track admin status ---
# This dictionary will be shared across all requests and threads
admin_status = {
    "admin_name": None,
    # Possible statuses: IDLE, CONFIGURING, CONFIGURED, STARTING, HOSTING
    "status": "IDLE", 
    "hosting_active": False
}
# --------------------------------------------------


@app.route("/")
def index():
    # --- MODIFIED: Check the real status ---
    is_setup = admin_status["hosting_active"]
    return render_template("index.html", is_setup=is_setup)


@app.route("/admin")
def admin_page():
    if "user" not in session:
        return redirect(url_for("login_page"))
    # --- MODIFIED: Pass the current status to the template ---
    return render_template(
        "admin.html", 
        ws_port=BROWSER_WS_PORT,
        current_status=admin_status["status"] # Pass status to template
    )

# --- NEW: Admin action route to start configuration ---
@app.route("/admin/configure", methods=["POST"])
def admin_configure():
    if "user" not in session:
        return redirect(url_for("login_page"))

    admin_name = session["user"]
    
    # 1. Set status to "Configuring"
    admin_status["admin_name"] = admin_name
    admin_status["status"] = "CONFIGURING"
    
    # 2. Simulate work
    print(f"[ADMIN] {admin_name} is configuring...")
    time.sleep(3) # Simulate 3-second configuration
    
    # 3. Set status to "Configured"
    admin_status["status"] = "CONFIGURED"
    print(f"[ADMIN] {admin_name} finished configuration.")
    
    return redirect(url_for("admin_page"))

# --- NEW: Admin action route to start hosting ---
@app.route("/admin/start_host", methods=["POST"])
def admin_start_host():
    if "user" not in session:
        return redirect(url_for("login_page"))
    
    # Only allow starting if configuration is done
    if admin_status["status"] != "CONFIGURED":
        # Handle error (e.g., flash a message)
        return redirect(url_for("admin_page"))

    admin_name = session["user"]
    
    # 1. Set status to "Starting"
    admin_status["admin_name"] = admin_name
    admin_status["status"] = "STARTING"
    
    # 2. Simulate work
    print(f"[ADMIN] {admin_name} is starting host...")
    time.sleep(3) # Simulate 3-second startup
    
    # 3. Set status to "Hosting"
    admin_status["status"] = "HOSTING"
    admin_status["hosting_active"] = True
    print(f"[ADMIN] {admin_name} is now hosting.")
    
    return redirect(url_for("admin_page"))


@app.route("/client")
def client_page():
    return render_template("client.html", ws_port=BROWSER_WS_PORT)


# --- NEW: Server-Sent Events (SSE) route for status updates ---
# --- NEW: Server-Sent Events (SSE) route for status updates ---
@app.route("/status_updates")
def status_updates():
    def generate_status():
        # Store the last status sent to avoid sending duplicates
        last_sent_status = None
        last_sent_admin = None  # <-- ADDED: Track admin name changes too

        while True:
            current_status = admin_status["status"]
            current_admin = admin_status["admin_name"]

            # --- MODIFIED: Check if status OR admin name changed ---
            if current_status != last_sent_status or current_admin != last_sent_admin:
                admin_name = current_admin
                message = ""
                
                # --- MODIFIED: New message logic ---
                if current_status == "IDLE":
                    if admin_name:
                        message = f"Admin '{admin_name}' login! Waiting for admin to start configuration..."
                    else:
                        message = "Waiting for admin login"
                elif current_status == "CONFIGURING":
                    message = f"admin '{admin_name}' is currently configuring"
                elif current_status == "CONFIGURED":
                    message = f"admin '{admin_name}' configured the setup successfully"
                elif current_status == "STARTING":
                    message = f"admin '{admin_name}' is starting hosting"
                elif current_status == "HOSTING":
                    message = f"you are connected to admin '{admin_name}'"
                
                # Format the data as an SSE message (data: json_string\n\n)
                data = json.dumps({"message": message})
                yield f"data: {data}\n\n"
                
                last_sent_status = current_status
                last_sent_admin = current_admin  # <-- ADDED: Update last admin name
            
            # Wait 1 second before checking again
            time.sleep(1)

    # Return a streaming response
    return Response(generate_status(), mimetype="text/event-stream")
    def generate_status():
        # Store the last status sent to avoid sending duplicates
        last_sent_status = None
        while True:
            current_status = admin_status["status"]
            
            # Only send an update if the status has changed
            if current_status != last_sent_status:
                admin_name = admin_status.get("admin_name", "admin") # Default name
                message = ""
                
                # Create the message text based on the status
                if current_status == "IDLE":
                    message = "Waiting for admin to start configuration..."
                elif current_status == "CONFIGURING":
                    message = f"admin '{admin_name}' is currently configuring"
                elif current_status == "CONFIGURED":
                    message = f"admin '{admin_name}' configured the setup successfully"
                elif current_status == "STARTING":
                    message = f"admin '{admin_name}' is starting hosting"
                elif current_status == "HOSTING":
                    message = f"you are connected to admin '{admin_name}'"
                
                # Format the data as an SSE message (data: json_string\n\n)
                data = json.dumps({"message": message})
                yield f"data: {data}\n\n"
                
                last_sent_status = current_status
            
            # Wait 1 second before checking again
            time.sleep(1)

    # Return a streaming response
    return Response(generate_status(), mimetype="text/event-stream")


@app.route("/register", methods=["GET", "POST"])
def register_page():
    if request.method == "POST":
        username = request.form["username"]
        password = request.form["password"]
        if register_admin(username, password):
            return redirect(url_for("login_page"))
        else:
            return "Username already exists!"
    return render_template("register.html")


@app.route("/login", methods=["GET", "POST"])
def login_page():
    if request.method == "POST":
        username = request.form["username"]
        password = request.form["password"]
        if verify_admin(username, password):
            session["user"] = username
            # --- MODIFIED: Set status on login ---
            # (In case server restarted while admin was logged in)
            if admin_status["status"] == "HOSTING":
                admin_status["admin_name"] = username
            else:
                # If not hosting, reset to idle
                admin_status["admin_name"] = username
                admin_status["status"] = "IDLE"
                admin_status["hosting_active"] = False

            return redirect(url_for("admin_page"))
        else:
            return "Invalid username or password"
    return render_template("login.html")


@app.route("/logout")
def logout_page():
    session.pop("user", None)
    # --- MODIFIED: Reset status on logout ---
    admin_status["admin_name"] = None
    admin_status["status"] = "IDLE"
    admin_status["hosting_active"] = False
    print("[ADMIN] Admin logged out, status reset.")
    return redirect(url_for("login_page"))


@app.route("/gallery")
def gallery_page():
    # --- MODIFIED: Check the real status ---
    is_setup = admin_status["hosting_active"]
    
    folders = []
    try:
        with os.scandir(GALLERY_PATH) as entries:
            folders = [entry.name for entry in entries if entry.is_dir()]
    except FileNotFoundError:
        print("No Gallery Folder Yet")
        pass

    return render_template(
        "gallery.html", 
        is_setup=is_setup, 
        ws_port=BROWSER_WS_PORT,
        folders=folders
    )


@app.route("/gallery/<string:foldername>")
def gallery_folder(foldername):
    # (No changes needed in this function)
    image_extensions = {'.jpg', '.png'}
    foldername = secure_filename(foldername)
    
    folder_path = os.path.join(GALLERY_PATH, foldername)

    if not os.path.isdir(folder_path):
        print("FOLDER: Does not exist")
        return redirect(url_for("gallery_page"))

    try:
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
    # --- MODIFIED: Protect the route ---
    if not admin_status["hosting_active"]:
        # If hosting is not active, redirect them to the client page
        # where they can see the status.
        print("[CLIENT] Denied access to /stream, hosting not active.")
        return redirect(url_for("client_page"))
        
    print("[CLIENT] Accessing /stream.")
    return render_template("stream.html", ws_port=BROWSER_WS_PORT)


@app.route("/video_feed")
def video_feed():
    # --- MODIFIED: Protect the video feed ---
    if not admin_status["hosting_active"]:
        print("[CLIENT] Denied access to /video_feed, hosting not active.")
        return "Hosting is not active.", 403 # Return an error
        
    return Response(generate_frames(), mimetype="multipart/x-mixed-replace; boundary=frame")


def run_webserver(port):
    print(f"[FLASK] running on 0.0.0.0:{port}")
    app.run(
        host="0.0.0.0",
        port=port,
        debug=True,
        threaded=True # threaded=True is CRITICAL for SSE and time.sleep()
    )


if __name__ == "__main__":
    run_webserver(5000)