import os
import time  # <-- ADDED: For simulating delays
import json  # <-- ADDED: For sending SSE data
import threading
from flask import Flask, Response, render_template, redirect, sessions, url_for, request, session, jsonify, request, flash, send_from_directory, abort
from pytest import Session
from werkzeug.utils import secure_filename
from urllib.parse import quote

from app import create_app, db
from app.models.admin import Admin
from app.models.gallery import Gallery
from app.models.session import Session
from app.services.auth_service import register_admin, verify_admin
from app.services.admin_status import AdminStatusManager
from app.utils.utils import find_esp_ip, check_esp_ws_connection

from image_processing import generate_frames, enable_archiving, disable_archiving
from config import BROWSER_WS_PORT, prototype_config
from background.prototype_manager import PrototypeManager
from background.image_generator import Archiver


# ---------------------------------------------------------------------
# Flask app initialization
# ---------------------------------------------------------------------
app = create_app()
app.secret_key = "polycast-creator_BatsiKuruSyaniOmit"

# # Initialize database (ensures tables exist)
# with app.app_context():
#     init_auth_db(app)

thread_manager = PrototypeManager()     # prototype background thread
archiver_manager = None

# Directory for gallery images
# GALLERY_PATH = os.path.join(app.static_folder, "gallery_images")
GALLERY_PATH = os.path.join(os.getcwd(), "archive")

# --- Shared state for admin status ---
admin_status = AdminStatusManager()
# ---------------------------------------------------------------------


# GLOBAL VARIABLES
@app.context_processor
def inject_nav_tabs():
    return dict(nav_tabs={
        'Stream': {'url': url_for('stream_page'), 'endpoints': ['client_page', 'stream_page', 'admin_page']},
        'Gallery': {'url': url_for('gallery_page'), 'endpoints': ['gallery_page', 'session_page', 'folderview_page']}
    })


@app.route("/")
def index():
    # --- MODIFIED: Check the real status ---
    is_setup = admin_status.get_field("hosting_active")
    return render_template("index.html", is_setup=is_setup)


@app.route("/admin")
def admin_page():
    if "user" not in session:
        return redirect(url_for("login_page"))
    
    # FETCH ADMIN
    admin_name = str(admin_status.get_field('admin_name'))
    
    current_admin = Admin.query.filter_by(username=admin_name).first()
    if not current_admin:
        return "Admin not found", 404
    
    # FETCH THE USER OBJECT: "current_admin.gallery" translated code
    # Temporary solutiuonn
    admin_galleries = Gallery.query.filter_by(admin_id=current_admin.id) \
                                .filter(Gallery.deleted_at.is_(None)) \
                                .order_by(Gallery.name) \
                                .all()

    gallery_list_for_json = [g.to_dict() for g in admin_galleries]
    
    return render_template(
        "admin.html", 
        ws_port=BROWSER_WS_PORT,
        current_status=admin_status.get_field("status"), 
        gallery_data=gallery_list_for_json 
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
    admin_status._update_state(
        status="CONFIGURING",
        admin_name=admin_name,
    )
    print(f"[ADMIN] {admin_name} is configuring with IP: {esp_ip}...")
    
    # Perform the actual WebSocket connection check
    is_connected = check_esp_ws_connection(ws_url_to_test)
    
    if is_connected:
        
        prototype_config.PROTOTYPE_IP = esp_ip  # Save the IP
        
        admin_status._update_state(
            status="CONFIGURED",
        )
        
        print(f"[ADMIN] {admin_name} finished configuration. Connection SUCCESS.")
        flash(f"Successfully connected to PolyCast at {esp_ip}!", "success")
        
        result = thread_manager.start()
        print("[PROTO] started" if result else "[PROTO] already running")
    
    else:
        # 4. FAILURE: Set status back to "IDLE"
        admin_status._update_state(
            status="IDLE",
        )
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
    global archiver_manager
    
    if "user" not in session:
        return redirect(url_for("login_page"))
    
    if admin_status.get_field("status") != "CONFIGURED":
        return redirect(url_for("admin_page"))

    # Initilize the data
    admin_name = session["user"]
    admin_id = session["id"]
    gallery_name = request.form.get('gallery_name')
    session_name = request.form.get('session_name')
    
    # Start creating new gallery and session
    admin_status._update_state(status="STARTING")
    print(f"[ADMIN] {admin_name} is starting host...")
    
    # Check if the gallery exists
    existing_gallery = Gallery.query.filter_by(name=gallery_name, admin_id=admin_id).first()
    if not existing_gallery:
        new_gallery = Gallery(
            name=gallery_name,
            admin_id=admin_id
        )
        db.session.add(new_gallery)
        db.session.commit()
        gallery_id = new_gallery.id
    else:
        gallery_id = existing_gallery.id
    
    # Check if session already exists for this gallery
    existing_session = Session.query.filter_by(name=session_name, gallery_id=gallery_id).first()
    if existing_session:
        print(f"[ADMIN] Session '{session_name}' already exists for gallery '{gallery_name}'. Skipping creation.")
        session_id = existing_session.id
    else:
        new_session = Session(
            name=session_name,
            gallery_id=gallery_id,
        )
        db.session.add(new_session)
        db.session.commit()
        session_id = new_session.id
    
    
    # Set status to "Hosting"
    admin_status._update_state(
        status="HOSTING", 
        hosting_active=True
    )
    print(f"[ADMIN] {admin_name} is now hosting.")
    
    archiver_manager = Archiver(
        initial_archive_path=os.path.join("archive", str(admin_name), str(gallery_id), str(session_id))
    )
    archiver_manager.start()
    
    # enable_archiving()
    
    return redirect(url_for("admin_page"))

@app.route('/endsession')
def endsession():
    global archiver_manager
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
        
        admin_status._update_state(
            status="IDLE",
            hosting_active=False
        )
        
        # FOR NOW, CLOSEE
        # archiver_manager.stop()
        
        print("[ADMIN] Admin End the session, status reset.")
        
    except Exception as e:
        print(f"Error stopping thread: {e}")
    
    return redirect(url_for('admin_page'))




# ADMIN CRUD OPERATIONS:
@app.route("/gallery/<int:gallery_id>/delete", methods=['POST'])
def delete_gallery(gallery_id):
    current_user_id = session['id']
    
    # Find the specific gallery or return a 404 error
    gallery = Gallery.query.get_or_404(gallery_id)

    # SECURITY CHECK: Ensure the gallery belongs to the current user
    if gallery.admin_id != current_user_id:
        abort(403) # Forbidden

    try:
        gallery.delete(commit=False)    # custom softdelete method
        db.session.commit()
        
        # Updated flash message for clarity
        flash('Gallery has been moved to the recycle bin.', 'success')
    except Exception as e:
        db.session.rollback()
        flash(f'An error occurred while moving to bin: {e}', 'danger')

    return redirect(url_for('admin_page'))

@app.route("/gallery/<int:gallery_id>/update", methods=['POST'])
def update_gallery_name(gallery_id):
    # 1. Check if user is logged in
    if "user" not in session:
        return redirect(url_for("login_page"))

    current_user_id = session['id']
    
    # 2. Find the gallery or return 404
    gallery = Gallery.query.get_or_404(gallery_id)

    # 3. SECURITY CHECK: Ensure the gallery belongs to the current user
    if gallery.admin_id != current_user_id:
        abort(403) # Forbidden

    # 4. Get the new name from the form
    new_name = request.form.get('gallery_name')

    # 5. Validate the new name
    if not new_name or len(new_name.strip()) == 0:
        flash('A gallery name is required.', 'error')
        return redirect(url_for('admin_page'))
    
    new_name = new_name.strip()

    # 6. Optional: Check if a gallery with that name already exists
    existing = Gallery.query.filter_by(name=new_name, admin_id=current_user_id).first()
    if existing and existing.id != gallery_id:
         flash('A gallery with this name already exists.', 'error')
         return redirect(url_for('admin_page'))

    # 7. Update the database
    try:
        gallery.name = new_name
        db.session.commit()
        flash('Gallery name updated successfully.', 'success')
    except Exception as e:
        db.session.rollback()
        flash(f'An error occurred while updating the name: {e}', 'danger')

    return redirect(url_for('admin_page'))

@app.route("/session/<int:session_id>/delete", methods=['POST'])
def delete_session(session_id):
    if "user" not in session:
        return redirect(url_for("login_page"))
    
    current_user_id = session['id']

    # 1. Find the session
    session_to_delete = Session.query.get_or_404(session_id)

    # 2. SECURITY CHECK: Check if the session's gallery belongs to the user
    if session_to_delete.gallery.admin_id != current_user_id:
        abort(403) # Forbidden

    try:
        # 3. Use your soft-delete method
        session_to_delete.delete(commit=False) # Or db.session.delete(session_to_delete)
        db.session.commit()
        flash('Session has been moved to the recycle bin.', 'success')
    except Exception as e:
        db.session.rollback()
        flash(f'An error occurred: {e}', 'danger')

    # 4. Redirect back to the page the user was on
    # request.referrer is the URL they just came from (the session list)
    return redirect(request.referrer or url_for('admin_page'))

@app.route("/session/<int:session_id>/update", methods=['POST'])
def update_session_name(session_id):
    # 1. Check if user is logged in
    if "user" not in session:
        return redirect(url_for("login_page"))

    current_user_id = session['id']
    
    # 2. Find the session or return 404
    session_to_update = Session.query.get_or_404(session_id)

    # 3. SECURITY CHECK: Ensure the session's gallery belongs to the current user
    if session_to_update.gallery.admin_id != current_user_id:
        abort(403) # Forbidden

    # 4. Get the new name from the form
    new_name = request.form.get('session_name')

    # 5. Validate the new name
    if not new_name or len(new_name.strip()) == 0:
        flash('A session name is required.', 'error')
        return redirect(request.referrer) # Redirect to the page they were on
    
    new_name = new_name.strip()

    # 6. Check if a session with that name already exists *in this gallery*
    existing = Session.query.filter_by(
        name=new_name, 
        gallery_id=session_to_update.gallery_id
    ).first()
    
    if existing and existing.id != session_id:
         flash('A session with this name already exists in this gallery.', 'error')
         return redirect(request.referrer)

    # 7. Update the database
    try:
        session_to_update.name = new_name
        db.session.commit()
        flash('Session name updated successfully.', 'success')
    except Exception as e:
        db.session.rollback()
        flash(f'An error occurred: {e}', 'danger')

    # Redirect back to the session list page
    return redirect(request.referrer or url_for('admin_page'))




@app.route("/client")
def client_page():
    return render_template("client.html", ws_port=BROWSER_WS_PORT)


# --- Server-Sent Events (SSE) route for status updates ---
@app.route("/status_updates")
def status_updates():
    def generate_status():
        while True:
            # Get the current status and admin name
            current_status = admin_status.get_field("status")
            current_admin = admin_status.get_field("admin_name")

            print(admin_status.get_status())

            # Default placeholders
            main_hook = ""
            status_detail = ""
            image_number = 1  # default

            # --- Status Handling ---
            if current_status == "IDLE":
                if current_admin:
                    # 2. IDLE (Admin Logged In)
                    main_hook = "Hang tight!"
                    status_detail = "The admin is online and preparing."
                    image_number = 2
                else:
                    # 1. IDLE (No Admin)
                    main_hook = "Ready to Connect"
                    status_detail = "Waiting for the admin to log in."
                    image_number = 1

            elif current_status == "CONFIGURING":
                # 3. CONFIGURING
                main_hook = "Setup in Progress"
                status_detail = "Admin is currently configuring the prototype."
                image_number = 3

            elif current_status == "CONFIGURED":
                # 4. CONFIGURED
                main_hook = "Almost There!"
                status_detail = "Configuration is done. Waiting for session to start."
                image_number = 4

            elif current_status == "STARTING":
                # 5. STARTING
                main_hook = "Starting Up..."
                status_detail = "Session is loading. This should only take a moment."
                image_number = 5

            elif current_status == "HOSTING":
                # 6. HOSTING
                main_hook = "You're Connected!"
                status_detail = "Successfully connected to the admin's live session."
                image_number = 6

            # Prepare SSE message as JSON
            data = json.dumps({
                "status": current_status,
                "main_hook": main_hook,
                "status_detail": status_detail,
                "image": f"images/{image_number}.png"
            })

            # Send as an SSE message
            yield f"data: {data}\n\n"

            admin_status.status_changed.wait()

    return Response(generate_status(), mimetype="text/event-stream")





@app.route("/register", methods=["GET", "POST"])
def register_page():
    """
    Handles the user registration page.
    Validates form data, checks for matching passwords,
    and registers the user.
    """
    error = None
    if request.method == "POST":
        username = request.form["username"]
        password = request.form["password"]
        confirm_password = request.form["confirm_password"]

        # --- Validation ---
        if not username or not password or not confirm_password:
            error = "All fields are required."
        elif password != confirm_password:
            error = "Passwords do not match!"
        else:
            # --- Try to register ---
            # Assuming register_admin(username, password) returns True on success, False on failure
            try:
                if register_admin(username, password):
                    # Use flash for success message on redirect
                    flash("Registration successful! Please log in.", "success")
                    return redirect(url_for("login_page"))
                else:
                    error = "Username already exists!"
            except Exception as e:
                # Catch any other potential errors during registration
                print(f"Error during registration: {e}") # Log the error
                error = "An unexpected error occurred. Please try again."
    
    # On a GET request or if an error occurred during POST, render the register page
    return render_template("auth/register.html", error=error)


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
                return render_template("auth/login.html")
            
            # Login was successful
            session["id"] = user.id
            session["user"] = user.username
            return redirect(url_for("admin_page"))
        else:
            return "Invalid username or password"

    return render_template("auth/login.html")


@app.route("/logout")
def logout_page():
    global archiver_manager
    
    session.pop("user", None)
    session.pop("id", None)
    
    # End the prototype ws thread, if the session still running
    result = thread_manager.stop()
    print("[PROTO] stopped" if result else "[PROTO] not running")
    
    admin_status.reset()   # Reset status on logout
    
    print("[ADMIN] Admin logged out, status reset.")
    
    # archiver_manager.stop()
    
    return redirect(url_for("login_page"))




@app.route("/gallery")
def gallery_page():
    """
    List all galleries belonging to the logged-in admin,
    only showing galleries that exist in the database and are not soft-deleted.
    """

    # FETCH ADMIN
    admin_name = str(admin_status.get_field('admin_name'))
    
    current_admin = Admin.query.filter_by(username=admin_name).first()
    if not current_admin:
        return "Admin not found", 404
    
    # FETCH galleries from database, excluding soft-deleted ones
    admin_galleries = Gallery.query.filter_by(admin_id=current_admin.id) \
                                   .filter(Gallery.deleted_at.is_(None)) \
                                   .order_by(Gallery.name) \
                                   .all()

    # Check if the folder exists on disk using the gallery ID
    user_filepath = os.path.join(GALLERY_PATH, str(current_admin.username))
    eligible_folders = []
    for g in admin_galleries:
        folder_path = os.path.join(user_filepath, str(g.id))
        print(folder_path)  # Debug: see which paths are being checked
        if os.path.isdir(folder_path):
            eligible_folders.append(g.name)  # Keep name for display

    is_setup = admin_status.get_field("hosting_active")

    return render_template(
        "gallery/gallery.html", 
        is_setup=is_setup, 
        ws_port=BROWSER_WS_PORT,
        folders=eligible_folders
    )


@app.route("/gallery/<string:galleryname>")
def session_page(galleryname):
    """
    List all sessions within the selected gallery
    """
    
    # FETCH ADMIN
    admin_name = str(admin_status.get_field('admin_name'))
    
    current_admin = Admin.query.filter_by(username=admin_name).first()
    if not current_admin:
        return "Admin not found", 404
    
    # CHECK if user is currently logged in and is the admin using session
    if 'user' in session:
        is_current_user = session['user'] == admin_name
    else:
        is_current_user = False  # session key doesn't exist
    
    # FETCH the gallery by ID, make sure it belongs to this admin and is not soft-deleted
    gallery = Gallery.query.filter_by(name=galleryname, admin_id=current_admin.id) \
                           .filter(Gallery.deleted_at.is_(None)) \
                           .first()
    if not gallery:
        return "Gallery not found or deleted", 404

    # Path to the gallery folder using gallery ID
    gallery_filepath = os.path.join(GALLERY_PATH, admin_name, str(gallery.id))

    # FETCH sessions from database, only not soft-deleted
    sessions_db = Session.query.filter_by(gallery_id=gallery.id) \
                               .filter(Session.deleted_at.is_(None)) \
                               .all()

    # Only include sessions whose folders exist on disk
    sessions = []
    for s in sessions_db:
        folder_path = os.path.join(gallery_filepath, str(s.id))
        if os.path.isdir(folder_path):
            sessions.append({'id': s.id, 'name': s.name})
            
    return render_template(
        "gallery/session.html",
        selected_gallery=gallery.name,
        sessions=sessions,
        is_current_user=is_current_user
    )


@app.route("/gallery/<string:galleryname>/<string:sessionname>")
def folderview_page(galleryname, sessionname):
    """
    Show all images within the selected folder
    """
    image_extensions = {'.jpg', '.png'}
    foldername = secure_filename(sessionname)
    admin_name = str(admin_status.get_field('admin_name'))
    
    # FETCH ADMIN using admin_name from admin_status
    admin_name = str(admin_status.get_field('admin_name'))
    current_admin = Admin.query.filter_by(username=admin_name).first()
    if not current_admin:
        return "Admin not found", 404
    
    # FETCH gallery and ensure it belongs to admin and is not soft-deleted
    gallery = Gallery.query.filter_by(name=galleryname, admin_id=current_admin.id) \
                           .filter(Gallery.deleted_at.is_(None)) \
                           .first()
    if not gallery:
        return "Gallery not found or deleted", 404

    # FETCH session and ensure it belongs to the gallery and is not soft-deleted
    session_obj = Session.query.filter_by(name=sessionname, gallery_id=gallery.id) \
                               .filter(Session.deleted_at.is_(None)) \
                               .first()
    
    if not session_obj:
        return "Session not found or deleted", 404
    
    # Path to the session folder using IDs
    folder_path = os.path.join(GALLERY_PATH, admin_name, str(gallery.id), str(session_obj.id))

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
    
    base_url = f"{admin_name}/{gallery.id}/{session_obj.id}"
    
    return render_template(
        "gallery/view.html",
        admin_name=admin_name,
        galleryname=gallery.name,
        sessionname=session_obj.name,
        images=images,
        base_url=base_url
    )


# BRIDGE ROUTE FOR CONNECTING ARCHIVE TO FLASK SERVER
@app.route("/gallery_images/<path:filename>")
def serve_gallery_image(filename):
    """
    Serve dynamically generated images from archive.
    filename: relative path inside <admin_name>/<gallery>/<session>/<image>
    """
    

    # Construct the full filesystem path safely
    requested_path = os.path.abspath(os.path.join(GALLERY_PATH, filename))
    safe_base = os.path.abspath(GALLERY_PATH) + os.sep

    # Security check: prevent directory traversal
    if not requested_path.startswith(safe_base):
        print(f"Forbidden attempt: {requested_path}")
        abort(403)

    if not os.path.isfile(requested_path):
        print(f"File not found: {requested_path}")
        abort(404)
    
    relative_path = os.path.relpath(requested_path, GALLERY_PATH).replace(os.sep, "/")
    return send_from_directory(GALLERY_PATH, relative_path)




@app.route("/stream")
def stream_page():
    if not admin_status.get_field("hosting_active"):
        # If hosting is not active, redirect them to the client page
        print("[CLIENT] Denied access to /stream, hosting not active.")
        return redirect(url_for("client_page"))
        
    print("[CLIENT] Accessing /stream.")
    return render_template("stream.html", ws_port=BROWSER_WS_PORT)


@app.route("/video_feed")
def video_feed():
    if not admin_status.get_field("hosting_active"):
        print("[CLIENT] Denied access to /video_feed, hosting not active.")
        return "Hosting is not active.", 403 # Return an error
        
    return Response(generate_frames(), mimetype="multipart/x-mixed-replace; boundary=frame")


def run_webserver(port):
    print(f"[FLASK] running on 0.0.0.0:{port}")
    app.run(
        host="0.0.0.0",
        port=port,
        debug=True,
        threaded=True # CRITICAL for SSE
    )


if __name__ == "__main__":
    run_webserver(5000)