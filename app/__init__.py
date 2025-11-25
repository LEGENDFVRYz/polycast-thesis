# app/__init__.py
from flask import Flask
from flask_sqlalchemy import SQLAlchemy
from sqlalchemy import event
from sqlalchemy.engine import Engine
from sqlite3 import Connection as SQLite3Connection


@event.listens_for(Engine, "connect")
def _set_sqlite_pragma(dbapi_connection, connection_record):
    if isinstance(dbapi_connection, SQLite3Connection):
        cursor = dbapi_connection.cursor()
        cursor.execute("PRAGMA foreign_keys=ON;")
        cursor.close()

# Initialize SQLAlchemy globally (used in models and services)
db = SQLAlchemy()


def create_app():
    """Application factory for creating Flask app instances."""
    app = Flask(__name__, instance_relative_config=True)

    # --- Configuration ---
    app.config.from_mapping(
        SECRET_KEY="polycast-creator_BatsiKuruSyaniOmit",
        SQLALCHEMY_DATABASE_URI="sqlite:///../instance/app.db",
        SQLALCHEMY_TRACK_MODIFICATIONS=False
    )

    # --- Initialize database ---
    db.init_app(app)
    
    # --- Import models so SQLAlchemy knows them ---
    from app.models import admin, gallery, session, note


    return app
