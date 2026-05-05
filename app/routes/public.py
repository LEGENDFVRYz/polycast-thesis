import shutil
from flask import Blueprint, render_template, jsonify
from sqlalchemy import text
from app.globals import admin_status
from app import db
from background.image_generator import (
    get_stream_client_count,
    get_renderer_fps,
    get_last_encoded_ts,
)
from config import ARCHIVE_DIR


public_bp = Blueprint('public', __name__)


@public_bp.route("/health")
def health():
    return jsonify(status="ok"), 200


@public_bp.route("/api/status")
def api_status():
    db_ok = True
    try:
        db.session.execute(text("SELECT 1"))
    except Exception:
        db_ok = False

    try:
        free_mb = shutil.disk_usage(ARCHIVE_DIR).free // (1024 * 1024)
    except Exception:
        free_mb = None

    return jsonify(
        admin_name=admin_status.get_field("admin_name"),
        admin_status=admin_status.get_field("status"),
        hosting_active=admin_status.get_field("hosting_active"),
        stream_gallery=admin_status.get_field("stream_gallery"),
        stream_session=admin_status.get_field("stream_session"),
        last_activity_ts=admin_status.get_field("last_activity"),
        stream_clients=get_stream_client_count(),
        renderer_fps=round(get_renderer_fps(), 2),
        last_encoded_ts=get_last_encoded_ts(),
        disk_free_mb=free_mb,
        db_ok=db_ok,
    )


@public_bp.route("/")
def index():
    is_setup = admin_status.get_field("hosting_active")
    return render_template("index.html", is_setup=is_setup)

@public_bp.route("/about")
def about_page():
    is_setup = admin_status.get_field("hosting_active")
    return render_template("web/about.html", is_setup=is_setup)

@public_bp.route("/privacy-policy")
def privacy_page():
    return render_template("web/privacy.html")

@public_bp.route("/terms-and-conditions")
def terms_page():
    return render_template("web/conditions.html")


@public_bp.route("/preview/error")
def error_preview_page():
    return render_template(
        "web/error.html",
        error_code=404,
        error_title="The Page Went Missing.",
        error_message="Our PolyCast couldn't find this whiteboard space. The content you are looking for may have been moved, renamed, or archived elsewhere. Please verify the URL or return to the main hub to continue your session.",
    )


@public_bp.route("/preview/error500")
def error500_preview_page():
    return render_template(
        "web/error500.html",
        error_code=500,
        error_title="A Glitch in the Archive.",
        error_message="We're experiencing an unexpected internal archiving issue. Our system admin team has been notified and is working on the case. We apologize for the interruption. Please try again shortly.",
    )