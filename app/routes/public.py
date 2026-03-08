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