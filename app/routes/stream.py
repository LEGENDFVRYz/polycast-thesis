import json
from flask import Blueprint, render_template, redirect, url_for, session, Response
from config import BROWSER_WS_PORT
from background.image_generator import generate_frames

# Import globals
import app.globals as g

stream_bp = Blueprint('stream', __name__)



# ---------------------------------------------------------------------
# Client Panel Routes
# - Waiting page for the clients if the stream is not yet setup
# ---------------------------------------------------------------------
@stream_bp.route("/client")
def client_page():
    current_status = g.admin_status.get_field("status")
    current_admin = g.admin_status.get_field("admin_name")
    
    # Default fallback
    image_number = 1 
    
    if current_status == "IDLE":
        image_number = 2 if current_admin else 1    # whether no admin or admin has logged in
    elif current_status == "CONFIGURING":
        image_number = 3
    elif current_status == "CONFIGURED":
        image_number = 4
    elif current_status == "STARTING":
        image_number = 5
    elif current_status == "HOSTING":
        image_number = 6
    
    return render_template("client.html", initial_image=f"{image_number}.webp", ws_port=BROWSER_WS_PORT)


@stream_bp.route("/status_updates")
def status_updates():
    """
    Server-Sent Events
    - Allows client user to know the public status of the prototype and admin activity
    - Return, status visualization and notice messages
    """
    def generate_status():
        while True:
            # Get the current status and admin name
            current_status = g.admin_status.get_field("status")
            current_admin = g.admin_status.get_field("admin_name")
            print(g.admin_status.get_status())

            # Default placeholders
            main_hook = ""
            status_detail = ""
            image_number = 1  # default

            # --- Status Handler ---
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


            # Prepare SSE message (JSON)
            data = json.dumps({
                "status": current_status,
                "main_hook": main_hook,
                "status_detail": status_detail,
                "image": f"images/{image_number}.webp"
            })

            # Send as an SSE message
            yield f"data: {data}\n\n"
            g.admin_status.status_changed.wait()

    return Response(generate_status(), mimetype="text/event-stream")



# ---------------------------------------------------------------------
# Stream Page Routes
# ---------------------------------------------------------------------
@stream_bp.route("/stream")
def index():
    
    # If no host, redirect them to the client page
    if not g.admin_status.get_field("hosting_active"):
        
        is_current_user = ('user' in session) and (session['user'] == str(g.admin_status.get_field('admin_name')))
        
        if is_current_user:
            return redirect(url_for("admin.index"))
        else:
            return redirect(url_for("stream.client_page"))
        
    print("[CLIENT] Accessing /stream.")
    return render_template("stream.html", ws_port=BROWSER_WS_PORT)


@stream_bp.route("/video_feed")
def video_feed():
    """Archive Notes Streamline"""
    
    if not g.admin_status.get_field("hosting_active"):
        print("[CLIENT] Access denied to /video_feed, no host available.")
        return "Hosting is not active.", 403 
    
    return Response(generate_frames(), mimetype="multipart/x-mixed-replace; boundary=frame")