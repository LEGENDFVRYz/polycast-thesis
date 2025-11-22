import os
import json
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

# from image_processing import generate_frames, enable_archiving, disable_archiving
from config import BROWSER_WS_PORT, prototype_config
from background.prototype_manager import PrototypeManager
from background.image_generator import Archiver, generate_frames


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



# ---------------------------------------------------------------------
# GLOBAL VARIABLES
# ---------------------------------------------------------------------
def is_current_user_admin(admin_status):
    """
    Returns True if the current session user is the admin.
    """
    admin_name = admin_status.get_field('admin_name') if admin_status else None
    current_user = session.get("user")
    return admin_name is not None and current_user == admin_name


# GLOBAL VARIABLES
@app.context_processor
def inject_is_admin():
    admin_name = admin_status.get_field('admin_name') if admin_status else None
    
    return dict(is_admin=is_current_user_admin(admin_status), admin_operator=admin_name)



# ---------------------------------------------------------------------
# WEB ROUTES
# ---------------------------------------------------------------------
@app.route("/")
def index():
    # --- MODIFIED: Check the real status ---
    is_setup = admin_status.get_field("hosting_active")
    return render_template("index.html", is_setup=is_setup)



@app.route("/admin")
def admin_page():
    # Check if user is logged in
    if "user" not in session:
        return redirect(url_for("login_page"))
    
    # FETCH ADMIN
    admin_name = str(admin_status.get_field('admin_name'))
    current_admin = Admin.query.filter_by(username=admin_name).first()
    
    gallery_list_for_json = []

    if current_admin:
        # FETCH galleries for the admin, excluding soft-deleted ones
        admin_galleries = (
            Gallery.query
            .filter_by(admin_id=current_admin.id)
            .filter(Gallery.deleted_at.is_(None))
            .order_by(Gallery.name)
            .all()
        )
        gallery_list_for_json = [g.to_dict() for g in admin_galleries]
        
    

    return render_template(
        "admin.html",
        ws_port=BROWSER_WS_PORT,
        prototype_ip=prototype_config.PROTOTYPE_IP,
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
    
    # --- Authentication Check ---
    if "user" not in session:
        flash("You must be logged in to perform this action.", "error")
        return redirect(url_for("login_page"))
    
    # --- Status Check ---
    if admin_status.get_field("status") != "CONFIGURED":
        flash("System is not in a 'CONFIGURED' state yet", "warning")
        return redirect(url_for("admin_page"))

    # --- Form Data Validation ---
    admin_name = session["user"]
    admin_id = session["id"]
    gallery_name = request.form.get('gallery_name', '').strip()
    session_name = request.form.get('session_name', '').strip()
    
    if not gallery_name:
        flash("Gallery name is required.", "error")
        return redirect(url_for("admin_page"))

    if not session_name:
        flash("Session name is required.", "error")
        return redirect(url_for("admin_page"))
    
    
    # Start creating new gallery and session
    print(f"[ADMIN] {admin_name} is starting host...")
    admin_status._update_state(status="STARTING")
    
    # --- Gallery Logic ---
    existing_gallery = Gallery.query.filter_by(name=gallery_name, admin_id=admin_id).first()
    if not existing_gallery:
        try:
            new_gallery = Gallery(
                name=gallery_name,
                admin_id=admin_id
            )
            db.session.add(new_gallery)
            db.session.commit()
            gallery_id = new_gallery.id
            flash(f"New gallery '{gallery_name}' created.", "notice")
        except Exception as e:
            db.session.rollback()
            print(f"[ADMIN] Error creating gallery: {e}")
            flash(f"Error creating gallery: {e}", "error")
            admin_status._update_state(status="CONFIGURED") # Rollback status
            return redirect(url_for("admin_page"))
    else:
        gallery_id = existing_gallery.id
        flash(f"Connecting to existing gallery '{gallery_name}'.", "notice")
    
    # --- Session Logic (Allow existing) ---
    existing_session = Session.query.filter_by(name=session_name, gallery_id=gallery_id).first()
    if existing_session:
        session_id = existing_session.id
        print(f"[ADMIN] Connecting to existing session '{session_name}' for gallery '{gallery_name}'.")
        flash(f"Connecting to existing session '{session_name}'.", "notice")
    else:
        try:
            new_session = Session(
                name=session_name,
                gallery_id=gallery_id,
            )
            db.session.add(new_session)
            db.session.commit()
            session_id = new_session.id
            flash(f"New session '{session_name}' created.", "notice")
        except Exception as e:
            db.session.rollback()
            print(f"[ADMIN] Error creating session: {e}")
            flash(f"Error creating session: {e}", "error")
            admin_status._update_state(status="CONFIGURED") # Rollback status
            return redirect(url_for("admin_page"))
    
    
    # Set status to "Hosting"
    print(f"[ADMIN] {admin_name} is now hosting.")
    admin_status._update_state(
        status="HOSTING", 
        hosting_active=True
    )
    session["gname"] = gallery_name
    session["sname"] = session_name
    
    try:
        archive_path = os.path.join("archive", str(admin_name), str(gallery_id), str(session_id))
        archiver_manager = Archiver(
            initial_archive_path=archive_path
        )
        archiver_manager.start()
        print(f"[ADMIN] Archiver started for path: {archive_path}")
    except Exception as e:
        print(f"[ADMIN] CRITICAL: Failed to start Archiver: {e}")
        flash(f"Hosting started, but archiver failed to initialize: {e}", "error")
    
    flash(f"Successfully started hosting session '{session_name}'.", "success")
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
        archiver_manager.stop()
        session.pop("gname", None)
        session.pop("sname", None)
        
        print("[ADMIN] Admin End the session, status reset.")
        
    except Exception as e:
        print(f"Error stopping thread: {e}")
    
    return redirect(url_for('admin_page'))




# ADMIN CRUD OPERATIONS:
@app.route("/gallery/<int:gallery_id>/delete", methods=['POST'])
def delete_gallery(gallery_id):
    
    if "user" not in session:
        flash("You must be admin in to perform this action.", "error")
        return redirect(url_for("login_page"))
    
    # Find the specific gallery or return a 404 error
    gallery = Gallery.query.get_or_404(gallery_id)

    # SECURITY CHECK: Ensure the gallery belongs to the current user
    current_user_id = session['id']
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
        flash("You must be admin in to perform this action.", "error")
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

@app.route("/gallery/<int:gallery_id>/recover", methods=['POST'])
def recover_gallery(gallery_id):
    # 1. Auth Check
    if "user" not in session:
        flash("You must be logged in to perform this action.", "error")
        return redirect(url_for("login_page"))
    
    # 2. Find the gallery
    # NOTE: If your global query filter hides deleted items, 'get_or_404' might fail here.
    # You might need to use: Gallery.query.with_deleted().filter_by(id=gallery_id).first_or_404()
    gallery = Gallery.query.get_or_404(gallery_id)

    # 3. SECURITY CHECK: Ensure the gallery belongs to the current user
    current_user_id = session['id']
    if gallery.admin_id != current_user_id:
        abort(403) # Forbidden

    try:
        # 4. Perform the restore
        # If you don't have a .restore() method, use: gallery.is_deleted = False
        if hasattr(gallery, 'restore'):
            gallery.restore(commit=False) 
        else:
            # Fallback if you are manually toggling a boolean
            gallery.is_deleted = False 
            
        db.session.commit()
        flash('Gallery has been successfully recovered from the bin.', 'success')
        
    except Exception as e:
        db.session.rollback()
        flash(f'An error occurred while recovering: {e}', 'danger')

    # Redirect to the Recycle Bin page or Admin page
    return redirect(request.referrer or url_for('admin_page'))

@app.route("/session/<int:session_id>/recover", methods=['POST'])
def recover_session(session_id):
    # 1. Auth Check
    if "user" not in session:
        return redirect(url_for("login_page"))
    
    current_user_id = session['id']

    # 2. Find the session
    session_to_recover = Session.query.get_or_404(session_id)

    # 3. SECURITY CHECK: Check ownership via the parent gallery
    if session_to_recover.gallery.admin_id != current_user_id:
        abort(403)

    # 4. LOGIC CHECK: Prevent recovering a session if the parent Gallery is still deleted
    # (Optional but recommended to prevent "orphan" sessions)
    if getattr(session_to_recover.gallery, 'is_deleted', False):
        flash('You cannot recover this session because its parent Gallery is in the bin.', 'error')
        return redirect(request.referrer)

    try:
        # 5. Perform the restore
        if hasattr(session_to_recover, 'restore'):
            session_to_recover.restore(commit=False)
        else:
            # Fallback manual toggle
            session_to_recover.is_deleted = False

        db.session.commit()
        flash('Session has been successfully recovered.', 'success')
        
    except Exception as e:
        db.session.rollback()
        flash(f'An error occurred: {e}', 'danger')

    return redirect(request.referrer or url_for('admin_page'))

@app.route("/gallery/<int:gallery_id>/force-delete", methods=['POST'])
def force_delete_gallery(gallery_id):
    # 1. Auth Check
    if "user" not in session:
        flash("You must be logged in to perform this action.", "error")
        return redirect(url_for("login_page"))
    
    # 2. Find the gallery
    gallery = Gallery.query.get_or_404(gallery_id)

    # 3. SECURITY CHECK: Ensure the gallery belongs to the current user
    current_user_id = session['id']
    if gallery.admin_id != current_user_id:
        abort(403) # Forbidden

    try:
        # 4. Perform Hard Delete
        # Note: If you store actual image files (locally or S3), 
        # you should call your file cleanup function here before deleting the DB row.
        
        db.session.delete(gallery) # This is the standard hard delete in SQLAlchemy
        db.session.commit()
        
        flash('Gallery has been permanently deleted.', 'success')
        
    except Exception as e:
        db.session.rollback()
        flash(f'An error occurred during permanent deletion: {e}', 'danger')

    # Redirect back to the Trash page (referrer)
    return redirect(request.referrer or url_for('admin_page'))

@app.route("/session/<int:session_id>/force-delete", methods=['POST'])
def force_delete_session(session_id):
    # 1. Auth Check
    if "user" not in session:
        return redirect(url_for("login_page"))
    
    current_user_id = session['id']

    # 2. Find the session
    session_to_delete = Session.query.get_or_404(session_id)

    # 3. SECURITY CHECK: Check ownership via the parent gallery
    if session_to_delete.gallery.admin_id != current_user_id:
        abort(403)

    try:
        # 4. Perform Hard Delete
        # TODO: Add logic here to delete actual physical files associated with this session
        
        db.session.delete(session_to_delete)
        db.session.commit()
        
        flash('Session has been permanently deleted.', 'success')
        
    except Exception as e:
        db.session.rollback()
        flash(f'An error occurred: {e}', 'danger')

    return redirect(request.referrer or url_for('admin_page'))

@app.route("/gallery/<int:gallery_id>/favorite", methods=['POST'])
def toggle_gallery_favorite(gallery_id):
    
    if "user" not in session:
        flash("You must be admin in to perform this action.", "error")
        return redirect(url_for("login_page"))
    
    current_user_id = session['id']
    gallery = Gallery.query.get_or_404(gallery_id)

    # Security Check
    if gallery.admin_id != current_user_id:
        abort(403)

    try:
        # Toggle the value
        gallery.is_favorite = not gallery.is_favorite
        db.session.commit()

        # 5. Feedback
        if gallery.is_favorite:
            flash(f'"{gallery.name}" added to favorites.', 'success')
        else:
            flash(f'"{gallery.name}" removed from favorites.', 'info')

    except Exception as e:
        db.session.rollback()
        flash(f'Error updating favorite status: {e}', 'danger')

    return redirect(request.referrer or url_for('admin_page'))





@app.route("/client")
def client_page():
    current_status = admin_status.get_field("status")
    current_admin = admin_status.get_field("admin_name")
    
    # Default fallback
    image_number = 1 

    if current_status == "IDLE":
        image_number = 2 if current_admin else 1
    elif current_status == "CONFIGURING":
        image_number = 3
    elif current_status == "CONFIGURED":
        image_number = 4
    elif current_status == "STARTING":
        image_number = 5
    elif current_status == "HOSTING":
        image_number = 6
    
    return render_template("client.html", initial_image=f"{image_number}.png", ws_port=BROWSER_WS_PORT)


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
    error = None

    if request.method == "POST":
        username = request.form.get("username")
        password = request.form.get("password")

        # --- Validation ---
        if not username or not password:
            error = "Both username and password are required."
        else:
            try:
                user = verify_admin(username, password)
                if user:
                    # Check if admin is already logged in
                    if not admin_status.login(user.username):
                        current_admin_name = admin_status.get_field('admin_name')
                        print(f"[STATUS CHECK] Login failed. Admin '{current_admin_name}' is already logged in.")
                        error = f"Admin '{current_admin_name}' is already logged in."
                    else:
                        # Login successful
                        session["id"] = user.id
                        session["user"] = user.username
                        return redirect(url_for("admin_page"))
                else:
                    error = "Invalid username or password."
            except Exception as e:
                # Catch unexpected errors
                print(f"Error during login: {e}")  # Log the error
                error = "An unexpected error occurred. Please try again."
    return render_template("auth/login.html", error=error)

@app.route("/logout")
def logout_page():
    global archiver_manager
    
    if not is_current_user_admin(admin_status):
        flash("Permission Denied", "error")
        return redirect(request.referrer) 
    
    session.pop("user", None)
    session.pop("id", None)
    
    # End the prototype ws thread, if the session still running
    result = thread_manager.stop()
    print("[PROTO] stopped" if result else "[PROTO] not running")
    
    admin_status.reset()   # Reset status on logout
    
    print("[ADMIN] Admin logged out, status reset.")
    
    if archiver_manager and archiver_manager.is_running():
        archiver_manager.stop()
        
    session.pop("gname", None)
    session.pop("sname", None)
    
    return redirect(url_for("login_page"))

@app.before_request
def check_admin_sync():
    if "user" not in session:
        return

    current_session_user = session["user"]
    server_side_admin = admin_status.get_field("admin_name")

    if server_side_admin is None:
        # SCENARIO: Server Restarted
        print(f"[SYNC] Server was restarted. Re-locking for {current_session_user}")
        admin_status.login(current_session_user)
    
    
    elif server_side_admin != current_session_user:
        # SCENARIO: There is new admin login
        print(f"[SYNC] Conflict. Server has {server_side_admin}, you are {current_session_user}. Logging out.")
        session.clear()
        flash("Session expired or another admin is active.", "error")
        return redirect(url_for("login_page"))

    admin_status.update_activity()




@app.route("/gallery")
def gallery_page():
    """
    List all galleries belonging to the logged-in admin.
    Filters based on the 'view' query parameter (all, favorites, trash)
    and only shows galleries that exist on disk.
    """

    current_view = request.args.get('view', 'all')

    # FETCH ADMIN
    admin_name = str(admin_status.get_field('admin_name'))
    current_admin = Admin.query.filter_by(username=admin_name).first()
 
    eligible_folders = []

    # If not current admin, just render empty gallery instead of redirect
    if not current_admin:
        return render_template(
            "gallery/gallery.html",
            is_setup=False,
            ws_port=BROWSER_WS_PORT,
            folders=[],
            active_view=current_view
        )

    # Base query for current admin
    base_query = Gallery.query.filter_by(admin_id=current_admin.id)

    # Apply view filter
    if current_view == 'favorites':
        base_query = base_query.filter_by(is_favorite=True).filter(Gallery.deleted_at.is_(None))
    elif current_view == 'trash':
        base_query = base_query.filter(Gallery.deleted_at.is_not(None))
    else:
        current_view = 'all'
        base_query = base_query.filter(Gallery.deleted_at.is_(None))

    # Fetch galleries
    admin_galleries = base_query.order_by(Gallery.name).all()

    # Check if gallery folder exists on disk
    user_filepath = os.path.join(GALLERY_PATH, str(current_admin.id))
    for g in admin_galleries:
        folder_path = os.path.join(user_filepath, str(g.id))
        if os.path.isdir(folder_path):
            # MODIFICATION: Append a dictionary with id and name
            eligible_folders.append({'id': g.id, 'name': g.name, 'is_favorite': g.is_favorite})

    is_setup = admin_status.get_field("hosting_active")

    # Render normally
    return render_template(
        "gallery/gallery.html",
        is_setup=is_setup,
        ws_port=BROWSER_WS_PORT,
        folders=eligible_folders,
        active_view=current_view
    )


@app.route("/gallery/<string:galleryname>")
def session_page(galleryname):
    """
    List all sessions within the selected gallery
    """
    
    current_view = request.args.get('view', 'all')
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
    gallery_filepath = os.path.join(GALLERY_PATH, str(current_admin.id), str(gallery.id))

    # BUILD BASE QUERY
    base_query = Session.query.filter_by(gallery_id=gallery.id)
    
    # 3. APPLY VIEW FILTER
    if current_view == 'trash':
        # Show only soft-deleted items
        sessions_db = base_query.filter(Session.deleted_at.is_not(None)).all()
    else:
        # Default: Show only active items
        current_view = 'all'
        sessions_db = base_query.filter(Session.deleted_at.is_(None)).all()
    

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
        active_view=current_view,
        is_current_user=is_current_user
    )


@app.route("/gallery/<string:galleryname>/<string:sessionname>")
def folderview_page(galleryname, sessionname):
    """
    Show all images within the selected folder
    """
    image_extensions = {'.jpg', '.png'}
    foldername = secure_filename(sessionname)
    
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
    folder_path = os.path.join(GALLERY_PATH, str(current_admin.id), str(gallery.id), str(session_obj.id))

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
    
    base_url = f"{current_admin.id}/{gallery.id}/{session_obj.id}"
    
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
    filename: relative path inside <admin_id>/<gallery>/<session>/<image>
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



# ---------------------------------------------------------------------
# API ROUTES
# ---------------------------------------------------------------------

@app.route("/api/gallery/<string:gallery_name>/sessions")
def get_gallery_sessions(gallery_name):
    """
    Return all session names for a given gallery (by name) as JSON.
    """
    admin_name = str(admin_status.get_field('admin_name'))
    current_admin = Admin.query.filter_by(username=admin_name).first()

    if not current_admin:
        return jsonify({"sessions": []})

    gallery = Gallery.query.filter_by(admin_id=current_admin.id, name=gallery_name).first()
    if not gallery:
        return jsonify({"sessions": []})

    sessions = Session.query.filter_by(gallery_id=gallery.id).order_by(Session.created_at.desc()).all()
    session_names = [s.name for s in sessions]

    return jsonify({"sessions": session_names})


# ---------------------------------------------------------------------
# UTILITIES
# ---------------------------------------------------------------------

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