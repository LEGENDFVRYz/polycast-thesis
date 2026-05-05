# app/__init__.py
from flask import Flask
from flask import Flask, session as flask_session
from flask_sqlalchemy import SQLAlchemy
from sqlalchemy import event
from sqlalchemy.engine import Engine
from sqlite3 import Connection as SQLite3Connection
import app.globals as g
from config import SECRET_KEY, SQLITE_BUSY_TIMEOUT_MS


@event.listens_for(Engine, "connect")
def _set_sqlite_pragma(dbapi_connection, connection_record):
    if isinstance(dbapi_connection, SQLite3Connection):
        cursor = dbapi_connection.cursor()
        cursor.execute("PRAGMA foreign_keys=ON;")
        cursor.execute("PRAGMA journal_mode=WAL;")
        cursor.execute("PRAGMA cache_size=-8000;")
        cursor.execute("PRAGMA synchronous=NORMAL;")
        cursor.execute(f"PRAGMA busy_timeout={SQLITE_BUSY_TIMEOUT_MS};")
        cursor.close()

# Initialize SQLAlchemy globally (used in models and services)
db = SQLAlchemy()


def create_app():
    """Application factory for creating Flask app instances."""
    app = Flask(__name__, instance_relative_config=True)

    # --- Configuration ---
    app.config.from_mapping(
        SECRET_KEY=SECRET_KEY,
        SQLALCHEMY_DATABASE_URI="sqlite:///../instance/app.db",
        SQLALCHEMY_TRACK_MODIFICATIONS=False
    )

    # --- Initialize database ---
    db.init_app(app)
    
    # --- Import models so SQLAlchemy knows them ---
    from app.models import admin, gallery, session, note
    
    
    @app.context_processor
    def inject_is_admin():
        admin_name = g.admin_status.get_field('admin_name') if g.admin_status else None
        current_user = flask_session.get("user")
        is_admin = admin_name is not None and current_user == admin_name
        return dict(is_admin=is_admin, admin_operator=admin_name)

    
    # Register all Blueprints
    from app.routes.public  import public_bp
    from app.routes.admin   import admin_bp
    from app.routes.gallery import gallery_bp
    from app.routes.stream  import stream_bp

    app.register_blueprint(public_bp)
    app.register_blueprint(admin_bp)
    app.register_blueprint(gallery_bp)
    app.register_blueprint(stream_bp)

    from flask import render_template

    @app.errorhandler(404)
    def error_404(e):
        return render_template(
            "_layouts/error.html",
            error_code=404,
            error_title="The Page Went Missing.",
            error_message="Our PolyCast couldn't find this whiteboard space. The content you are looking for may have been moved, renamed, or archived elsewhere. Please verify the URL or return to the main hub to continue your session.",
        ), 404

    @app.errorhandler(500)
    def error_500(e):
        return render_template(
            "_layouts/error.html",
            error_code=500,
            error_title="A Glitch in the Archive.",
            error_message="We're experiencing an unexpected internal archiving issue. Our system admin team has been notified and is working on the case. We apologize for the interruption. Please try again shortly.",
            is_server_error=True,
        ), 500

    @app.errorhandler(403)
    def error_403(e):
        return render_template(
            "_layouts/error.html",
            error_code=403,
            error_title="Access Restricted.",
            error_message="You don't have the necessary permissions to access this whiteboard space. If you believe this is a mistake, please contact your session administrator.",
        ), 403

    @app.errorhandler(401)
    def error_401(e):
        return render_template(
            "_layouts/error.html",
            error_code=401,
            error_title="Archive Not Found.",
            error_message="The gallery or session are missing. Either deleted by admin or expired.",
        ), 401

    @app.errorhandler(503)
    def error_503(e):
        return render_template(
            "_layouts/error.html",
            error_code=503,
            error_title="Service Temporarily Unavailable.",
            error_message="PolyCast is currently under maintenance or experiencing high load. Please wait a moment and try refreshing the page.",
        ), 503

    return app
