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

    return app
