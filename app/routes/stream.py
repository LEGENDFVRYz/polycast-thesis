import json
import os
from flask import Blueprint, render_template, redirect, url_for, session, Response
from config import BROWSER_WS_PORT, MJPEG_WIDTH, MJPEG_HEIGHT
from background.image_generator import (
    generate_frames,
    try_register_stream_client,
    unregister_stream_client,
)
from app import db
from app.models.admin import Admin
from app.models.gallery import Gallery
from app.models.session import Session as DBSession

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
    return render_template("stream/stream.html", ws_port=BROWSER_WS_PORT,
                           mjpeg_width=MJPEG_WIDTH, mjpeg_height=MJPEG_HEIGHT)


@stream_bp.route("/stream/playback")
def playback():
    import re
    IMAGE_EXTENSIONS = {'.jpg', '.jpeg', '.png'}
    CHECKPOINT_STEP = 10

    admin_name = str(g.admin_status.get_field('admin_name'))
    current_admin = Admin.query.filter_by(username=admin_name).first()

    def _empty(**kw):
        return render_template("stream/playback.html", images=[], base_url="", checkpoints=[], **kw)

    if not current_admin:
        return _empty(session_name=None, gallery_name=None, total=0)

    latest_session = (
        DBSession.query
        .join(Gallery, DBSession.gallery_id == Gallery.id)
        .filter(
            Gallery.admin_id == current_admin.id,
            Gallery.deleted_at.is_(None),
            DBSession.deleted_at.is_(None),
        )
        .order_by(DBSession.last_activity_at.desc(), DBSession.created_at.desc())
        .first()
    )

    if not latest_session:
        return _empty(session_name=None, gallery_name=None, total=0)

    folder_path = os.path.join(
        g.GALLERY_PATH,
        str(current_admin.id),
        str(latest_session.gallery_id),
        str(latest_session.id)
    )

    def _note_page_num(filename):
        m = re.match(r'^note_(\d+)\.[^.]+$', filename, re.IGNORECASE)
        return int(m.group(1)) if m else float('inf')

    images = []
    if os.path.isdir(folder_path):
        with os.scandir(folder_path) as entries:
            candidates = [
                e.name for e in entries
                if e.is_file() and os.path.splitext(e.name)[1].lower() in IMAGE_EXTENSIONS
            ]
        images = sorted(candidates, key=_note_page_num)

    total = len(images)

    # Checkpoints: index of every CHECKPOINT_STEP-th note (0-based), skip index 0
    checkpoints = [
        {"label": f"Page {i + 1}", "index": i}
        for i in range(0, total, CHECKPOINT_STEP)
        if i > 0
    ]

    base_url = f"{current_admin.id}/{latest_session.gallery_id}/{latest_session.id}"
    return render_template(
        "stream/playback.html",
        images=images,
        base_url=base_url,
        session_name=latest_session.name,
        gallery_name=latest_session.gallery.name,
        total=total,
        start_index=total - 1 if total > 0 else 0,
        checkpoints=checkpoints,
    )


@stream_bp.route("/video_feed")
def video_feed():
    """Archive Notes Streamline"""

    if not g.admin_status.get_field("hosting_active"):
        print("[CLIENT] Access denied to /video_feed, no host available.")
        return "Hosting is not active.", 403

    if not try_register_stream_client():
        return "Stream is at capacity. Please try again later.", 503

    def _gen():
        try:
            for chunk in generate_frames():
                yield chunk
        finally:
            unregister_stream_client()

    return Response(_gen(), mimetype="multipart/x-mixed-replace; boundary=frame")