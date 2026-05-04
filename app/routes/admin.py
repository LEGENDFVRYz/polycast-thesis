import os
from flask import Blueprint, render_template, redirect, url_for, request, session, jsonify, flash
from app import db
from app.models.admin import Admin
from app.models.gallery import Gallery

from app.models.session import Session as DBSession 
from app.services.auth_service import register_admin, verify_admin
from app.utils.utils import find_esp_ip, check_esp_ws_connection
from config import BROWSER_WS_PORT, prototype_config
from background.image_generator import Archiver

# Import globals
import app.globals as g

admin_bp = Blueprint('admin', __name__)



# ---------------------------------------------------------------------
# Admin Authentication Routes
# ---------------------------------------------------------------------
@admin_bp.route("/admin-register", methods=["GET", "POST"])
def register_page():

    error = None    # Store Error State for feedback
    
    if request.method == "POST":
        username = request.form["username"]
        password = request.form["password"]
        confirm_password = request.form["confirm_password"]

        # Validation: Checking if it user inputs are valid
        if not username or not password or not confirm_password:
            error = "All fields are required."
        elif password != confirm_password:
            error = "Passwords do not match!"
        else:
            try:
                if register_admin(username, password):
                    
                    flash("Registration successful! Please log in.", "success")
                    return redirect(url_for("admin.login_page"))
                else:
                    error = "Username already exists!"
                
            except Exception as e:
                print(f"Error during registration: {e}")
                error = "An unexpected error occurred. Please try again."
    
    return render_template("auth/register.html", error=error)


@admin_bp.route("/admin-login", methods=["GET", "POST"])
def login_page():
    
    error = None    # Store Error State for feedback

    if request.method == "POST":
        username = request.form.get("username")
        password = request.form.get("password")

        # Validation: Checking if it matches the record
        if not username or not password:
            error = "Both username and password are required."
            
        else:
            try:
                user = verify_admin(username, password)
                
                if user:
                    if not g.admin_status.login(user.username):
                        # If admin is already logged in
                        current_admin_name = g.admin_status.get_field('admin_name')
                        print(f"[STATUS CHECK] Login failed. Admin '{current_admin_name}' is already logged in.")
                        error = f"Admin '{current_admin_name}' is already logged in."
                        
                    else:
                        # If Login successful
                        session["id"] = user.id
                        session["user"] = user.username
                        return redirect(url_for("admin.index"))
                    
                else:
                    error = "Invalid username or password."
                    
            except Exception as e:
                print(f"Error during login: {e}")  # Log the error
                error = "An unexpected error occurred. Please try again."
    
    return render_template("auth/login.html", error=error)


@admin_bp.route("/admin-logout")
def logout():
    
    # HELPER: Check if the user is admin, and if the admin is the one who has access in the prototype
    admin_name = g.admin_status.get_field('admin_name') if g.admin_status else None
    current_user = session.get("user")
    is_admin = admin_name is not None and current_user == admin_name

    # EDGE CASES: If the user is not admin, and somehow able to use admin-logout  
    if not is_admin:
        flash("Permission Denied", "error")
        return redirect(request.referrer) 
    
    
    # Pop the admin user details
    session.pop("user", None)
    session.pop("id", None)
    
    # Halt all the operation if still running
    result = g.thread_manager.stop()
    print("[PROTO] stopped" if result else "[PROTO] not running")
    
    g.admin_status.reset()
    print("[ADMIN] Admin logged out, status reset.")
    
    if g.archiver_manager and g.archiver_manager.is_running():
        g.archiver_manager.stop()
    
    # Pop the admin running operation name (gallery and session)
    session.pop("gname", None)
    session.pop("sname", None)
    
    return redirect(url_for("public.index"))



# ---------------------------------------------------------------------
# Admin Panel Routes
# ---------------------------------------------------------------------
@admin_bp.route("/admin")
def index():
    # Validation: Reject the request if there is no login admin yet.
    if "user" not in session:
        return redirect(url_for("admin.login_page"))
    
    # Fetch the current admin
    admin_name = str(g.admin_status.get_field('admin_name'))
    current_admin = Admin.query.filter_by(username=admin_name).first()
    
    # Fetch all the galleries under the current admin
    gallery_list_for_json = []

    if current_admin:
        # Galleries of the admin, excluding soft-deleted ones
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
        # ws_port=BROWSER_WS_PORT,
        prototype_ip=prototype_config.SERIAL_PORT,
        current_status=g.admin_status.get_field("status"),
        gallery_data=gallery_list_for_json
    )


@admin_bp.route("/admin/configure", methods=["POST"])
def configure():
    """
    Admin Panel Prototype Configuration (PHASE 1/2):

    - Connects to the prototype using SERIAL_PORT and BAUD_RATE from .env
    - Verifies the port is accessible before launching the background thread
    - No user-selected port: the port is environment-configured
    """
    import serial as _serial

    if "user" not in session:
        flash("You must be logged in to perform this action.", "error")
        return redirect(url_for("admin.login_page"))

    admin_name = session["user"]
    port = prototype_config.SERIAL_PORT
    baud = prototype_config.BAUD_RATE

    g.admin_status._update_state(status="CONFIGURING", admin_name=admin_name)
    print(f"[ADMIN] {admin_name} checking prototype on {port} @ {baud}...")

    # --- Health check: verify the serial port opens before committing ---
    try:
        probe = _serial.Serial(port, baud, timeout=2)
        probe.close()
    except _serial.SerialException as e:
        print(f"[ADMIN] Prototype not reachable on {port}: {e}")
        g.admin_status._update_state(status="IDLE")
        flash(f"Prototype not found on {port}. Check the device is connected and SERIAL_PORT is correct.", "error")
        return redirect(url_for("admin.index"))
    except Exception as e:
        print(f"[ADMIN] Unexpected error probing {port}: {e}")
        g.admin_status._update_state(status="IDLE")
        flash(f"Unexpected error while connecting: {e}", "error")
        return redirect(url_for("admin.index"))

    # --- Port is reachable — start the background serial thread ---
    try:
        result = g.thread_manager.start()
        if result:
            print("[PROTO] Serial thread started successfully")
        else:
            print("[PROTO] Thread was already running")

        g.admin_status._update_state(status="CONFIGURED")
        print(f"[ADMIN] {admin_name} finished configuration on {port}.")
        flash(f"Prototype connected on {port}.", "success")

    except Exception as e:
        print(f"[ADMIN] Failed to start thread: {e}")
        g.admin_status._update_state(status="IDLE")
        flash(f"Failed to start prototype thread: {e}", "error")

    return redirect(url_for("admin.index"))


@admin_bp.route("/admin/start_host", methods=["POST"])
def start_host():
    """
    Admin Panel Prototype Configuration (PHASE 2/2):
    
    - Primary purpose: Handle the organization of the processed notes
    - Allows admins to set which gallery (main category) and session (sub-category) 
      the processed notes/images will go to.
    - Backed by a database, enabling us to put diff metadata for each note in future improvements
    
    """
    
    # Validation: Reject the request if there is no login admin yet.
    if "user" not in session:
        flash("You must be logged in to perform this action.", "error")
        return redirect(url_for("admin.login_page"))
    
    # EDGE CASES: If the admin user doesn't configure prototype correctly
    if g.admin_status.get_field("status") != "CONFIGURED":
        flash("System is not in a 'configured' yet", "warning")
        return redirect(url_for("admin.index"))


    # Fetch and process the admin inputs
    admin_name = session["user"]
    admin_id = session["id"]
    gallery_name = request.form.get('gallery_name', '').strip()
    session_name = request.form.get('session_name', '').strip()
    
    if not gallery_name or not session_name:
        # Ensure Valid Entries, else halt the operation
        flash("Gallery/Session names are required.", "error")
        return redirect(url_for("admin.index"))
    
    print(f"[ADMIN] {admin_name} is starting host...")
    g.admin_status._update_state(status="STARTING")
    

    # DATABASE OPERATION: Gallery Logic
    existing_gallery = Gallery.query.filter_by(name=gallery_name, admin_id=admin_id).first()
    
    if not existing_gallery:
        # Attempt to create new record, if the "gallery" folder is newly created
        try:
            new_gallery = Gallery(name=gallery_name, admin_id=admin_id)
            db.session.add(new_gallery)
            db.session.commit()
            
            gallery_id = new_gallery.id     # For New Session Creation
            flash(f"New gallery '{gallery_name}' created.", "notice")
            
        except Exception as e:
            db.session.rollback()
            
            flash(f"Error creating gallery: {e}", "error")
            g.admin_status._update_state(status="CONFIGURED")   # Rollback to old status
            return redirect(url_for("admin.index"))
        
    else:
        # If the gallery exist, connect to existing gallery
        gallery_id = existing_gallery.id
        flash(f"Connecting to existing gallery '{gallery_name}'.", "notice")
    

    # DATABASE OPERATION: Session Logic
    existing_session = DBSession.query.filter_by(name=session_name, gallery_id=gallery_id).first()
    
    if existing_session:
        # If the session exist, connect to existing session
        session_id = existing_session.id
        flash(f"Connecting to existing session '{session_name}'.", "notice")
        
    else:
        # Attempt to create new record, if the "gallery" folder is newly created
        try:
            new_session = DBSession(name=session_name, gallery_id=gallery_id)
            db.session.add(new_session)
            db.session.commit()
            
            session_id = new_session.id
            flash(f"New session '{session_name}' created.", "notice")
            
        except Exception as e:
            db.session.rollback()
            
            flash(f"Error creating session: {e}", "error")
            g.admin_status._update_state(status="CONFIGURED")    # Rollback to old status
            return redirect(url_for("admin.index"))
    
    
    # Fetch the notes location and update status to "hosting"
    g.admin_status._update_state(status="HOSTING", hosting_active=True)
    session["gname"] = gallery_name
    session["sname"] = session_name

    # Update last_activity_at so recent-sessions sorts correctly
    try:
        active_session = DBSession.query.get(session_id)
        if active_session:
            active_session.record_activity('ACTIVE')
            db.session.commit()
    except Exception as e:
        db.session.rollback()
        print(f"[ADMIN] Failed to record session activity: {e}")
    
    # Attempt to initialize the image generation
    try:
        archive_path = os.path.join("archive", str(admin_id), str(gallery_id), str(session_id))
        g.set_archiver_manager(Archiver(initial_archive_path=archive_path))
        g.archiver_manager.start()
    except Exception as e:
        flash(f"Hosting started, but archiver failed to initialize: {e}", "error")
        g.admin_status._update_state(status="CONFIGURED")    # Rollback to old status
        return redirect(url_for("admin.index"))
    
    
    flash(f"Successfully started hosting session '{session_name}'.", "success")
    return redirect(url_for("admin.index"))


@admin_bp.route('/endsession')
def endsession():
    try:
        # Attempt to halt all the background threads
        result = g.thread_manager.stop()
        print("[PROTO] stopped" if result else "[PROTO] not running")
        
        # Update the all status
        g.admin_status._update_state(status="IDLE", hosting_active=False)
        
        if g.archiver_manager:
            g.archiver_manager.stop()
        
        session.pop("gname", None)
        session.pop("sname", None)
        
        print("[ADMIN] Admin End the session, status reset.")

    except Exception as e:
        # EDGE CASE: Safety unexpected error handler
        print(f"Error stopping thread: {e}")
    
    
    return redirect(url_for('admin.index'))


# ---------------------------------------------------------------------
# Admin Panel Helper routes
# ---------------------------------------------------------------------

# Returns the env-configured serial port (port selection is no longer user-facing)
@admin_bp.route('/scan-ports')
def scan_ports():
    if 'user' not in session:
        return jsonify({'success': False, 'message': 'User not authenticated'}), 401
    return jsonify({'success': True, 'port': prototype_config.SERIAL_PORT})


# Admin Authentication Synchronization
# - Method for recovering access to rightful admin incase of uncertainty
# - Ensuring that only one admin can run the prototype/system
@admin_bp.before_app_request
def check_admin_sync():

    # Validation: Reject the request if there is no login admin yet.
    if "user" not in session:
        return None

    current_session_user = session["user"]
    server_side_admin = g.admin_status.get_field("admin_name")

    if server_side_admin is None:
        # EDGE CASES: Server Restarted Unexpectedly (no conflicts)
        g.admin_status.login(current_session_user)
        print(f"[SYNC] Server was restarted. Re-locking for {current_session_user}")
        
    elif server_side_admin != current_session_user:
        # EDGE CASES: Server has already run by some admin, Auth conflict
        session.clear()
        print(f"[SYNC] Auth Conflict: Server currently running by {server_side_admin}, you are {current_session_user}. Session Forcely Out.")
        flash("Session Expired, Server currently running by {server_side_admin}.", "error")
        return redirect(url_for("admin.login_page"))

    # Update the Global States
    g.admin_status.update_activity()
