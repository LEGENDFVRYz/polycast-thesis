from flask import Blueprint, render_template
from app.globals import admin_status


public_bp = Blueprint('public', __name__)


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