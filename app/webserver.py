import os
import time  # <-- ADDED: For simulating delays
import json  # <-- ADDED: For sending SSE data
import threading
from flask import Flask, Response, render_template, redirect, url_for, request, session, jsonify, request, flash
from pytest import Session
from werkzeug.utils import secure_filename

from app import create_app, db
from app.models.admin import Admin
from app.models.gallery import Gallery
from app.models.session import Session
from app.services.auth_service import register_admin, verify_admin
from app.services.admin_status import AdminStatusManager
from app.utils.utils import find_esp_ip, check_esp_ws_connection

from image_processing import generate_frames
from config import BROWSER_WS_PORT, prototype_config
from background.prototype_manager import PrototypeManager


# ---------------------------------------------------------------------
# Flask app initialization
# ---------------------------------------------------------------------
app = create_app()
app.secret_key = "polycast-creator_BatsiKuruSyaniOmit"

# # Initialize database (ensures tables exist)
# with app.app_context():
#     init_auth_db(app)

thread_manager = PrototypeManager()     # prototype background thread

# Directory for gallery images
GALLERY_PATH = os.path.join(app.static_folder, "gallery_images")

# --- Shared state for admin status ---
admin_status = AdminStatusManager()
# ---------------------------------------------------------------------


@app.route("/")
def index():
    # --- MODIFIED: Check the real status ---
    is_setup = admin_status.get_field("hosting_active")
    return render_template("index.html", is_setup=is_setup)


@app.route("/admin")
def admin_page():
    if "user" not in session:
        return redirect(url_for("login_page"))
    
    # FETCH GALLERY DATA
    current_admin = Admin.query.get(session["id"])
    admin_galleries = current_admin.gallery
    
    # gallery_data = [g.to_dict() for g in admin_galleries]   # ORM -> List(Dict())
    # gallery_json = json.dumps(gallery_data)
    
    gallery_list = [g.get_gallery_name() for g in admin_galleries]
    
    return render_template(
        "admin.html", 
        ws_port=BROWSER_WS_PORT,
        current_status=admin_status.get_field("status"),      # Pass status to template
        gallery_data=gallery_list
    )

# Admin Configurantion:
@app.route("/admin/configure", methods=["POST"])
def admin_configure():
    if "user" not in session:
        return redirect(url_for("login_page"))

    admin_name = session["user"]
    esp_ip = request.form.get('esp_ip')

    if not esp_ip:
        flash("No IP address was provided.", "error")
        return redirect(url_for("admin_page"))

    ws_url_to_test = f"ws://{esp_ip}/ws"


    # Set status to "Configuring..."
    admin_status["admin_name"] = admin_name
    admin_status["status"] = "CONFIGURING"
    print(f"[ADMIN] {admin_name} is configuring with IP: {esp_ip}...")
    
    # Perform the actual WebSocket connection check
    is_connected = check_esp_ws_connection(ws_url_to_test)
    
    if is_connected:
        
        prototype_config.PROTOTYPE_IP = esp_ip  # Save the IP
        
        admin_status["status"] = "CONFIGURED"
        print(f"[ADMIN] {admin_name} finished configuration. Connection SUCCESS.")
        flash(f"Successfully connected to PolyCast at {esp_ip}!", "success")
        
        # global prototype_reader_thread
        # if prototype_reader_thread and prototype_reader_thread.is_alive():
        #     print("Thread already running.")
        #     return redirect(url_for("admin_page"))

        # prototype_reader_thread_stop_event.clear()  # reset stop signal
        # prototype_reader_thread = threading.Thread(target=ws_client_thread, daemon=True)
        # prototype_reader_thread.start()
        
        result = thread_manager.start()
        print("[PROTO] started" if result else "[PROTO] already running")
    
    else:
        # 4. FAILURE: Set status back to "IDLE"
        admin_status["status"] = "IDLE"
        print(f"[ADMIN] {admin_name} configuration FAILED. Could not connect.")
        
        flash(f"Failed to connect to PolyCast at {esp_ip}. Check IP and network.", "error")
        
        prototype_config.PROTOTYPE_IP = None
    
    return redirect(url_for("admin_page"))

# Admin Configurantion Helper: API for scanning IP
@app.route('/scan-for-ip')
def scan_for_ip_route():
    if 'user' not in session:
        # If not logged in, return a JSON error, NOT a redirect
        return jsonify({'success': False, 'message': 'User not authenticated'}), 401
    
    ip = find_esp_ip(timeout=10) 
    
    if ip:
        return jsonify({'success': True, 'ip': ip})
    else:
        return jsonify({'success': False, 'message': 'Scan timed out. No device found.'}), 404

# --- NEW: Admin action route to start hosting ---
@app.route("/admin/start_host", methods=["POST"])
def admin_start_host():
    if "user" not in session:
        return redirect(url_for("login_page"))
    
    # Only allow starting if configuration is done
    if admin_status["status"] != "CONFIGURED":
        # Handle error (e.g., flash a message)
        return redirect(url_for("admin_page"))

    # CREATE GALLERY DATA
    admin_name = session["user"]
    admin_id = session["id"]
    gallery_name = request.form.get('gallery_name')
    session_name = request.form.get('session_name')
    
    # 1. Set status to "Starting"
    admin_status["admin_name"] = admin_name
    admin_status["status"] = "STARTING"
    
    # 2. Create a new gallery
    print(f"[ADMIN] {admin_name} is starting host...")
    
    existing_gallery = Gallery.query.filter_by(name=gallery_name).first()
    if not existing_gallery:
        new_gallery = Gallery(
            name=gallery_name,
            admin_id=admin_id
        )
        db.session.add(new_gallery)
        db.session.commit()
    
    # 2.5 Create a new session
    gallery_id = Gallery.query.filter_by(name=gallery_name).first().id
    new_session = Session(
        name=session_name,
        gallery_id=gallery_id,
    )
    db.session.add(new_session)
    db.session.commit()
    
    
    # 3. Set status to "Hosting"
    admin_status["status"] = "HOSTING"
    admin_status["hosting_active"] = True
    print(f"[ADMIN] {admin_name} is now hosting.")
    
    return redirect(url_for("admin_page"))

@app.route('/endsession')
def endsession():
    # global prototype_reader_thread # <-- Add this to modify the global var
    
    # if not prototype_reader_thread or not prototype_reader_thread.is_alive():
    #     print("No active thread to stop")
    #     return redirect(url_for('admin_page'))

    try:
        print("Signaling thread to stop...")
        # prototype_reader_thread_stop_event.set()  # 1. Signal the while loop to stop

        # if prototype_ws_app:
        #     print("Closing websocket connection...")
        #     prototype_ws_app.close()          # 2. This unblocks run_forever()

        # 3. Wait for the thread to actually finish
        # prototype_reader_thread.join(timeout=2.0)
        
        result = thread_manager.stop()
        print("[PROTO] stopped" if result else "[PROTO] not running")
        
        admin_status["status"] = "IDLE"
        admin_status["hosting_active"] = False
        print("[ADMIN] Admin End the session, status reset.")
        
    except Exception as e:
        print(f"Error stopping thread: {e}")
        
    return redirect(url_for('admin_page'))


@app.route("/client")
def client_page():
    return render_template("client.html", ws_port=BROWSER_WS_PORT)


# --- Server-Sent Events (SSE) route for status updates ---
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
        
        user = verify_admin(username, password)
        if user:
            if not admin_status.login(user.username):
                current_admin_name = admin_status.get_field('admin_name')
                print(f"[STATUS CHECK] Login failed. Admin '{current_admin_name}' is already logged in.")
                return render_template("login.html")
            
            # Login was successful
            session["id"] = user.id
            session["user"] = user.username
            return redirect(url_for("admin_page"))
        else:
            return "Invalid username or password"
        
    return render_template("login.html")


@app.route("/logout")
def logout_page():
    session.pop("user", None)
    session.pop("id", None)
    
    # End the prototype ws thread, if the session still running
    result = thread_manager.stop()
    print("[PROTO] stopped" if result else "[PROTO] not running")
    
    admin_status.reset()   # Reset status on logout
    
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