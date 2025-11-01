# app/__init__.py
from flask import Flask
from flask_sqlalchemy import SQLAlchemy
import os

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
    from app.models import admin, gallery, session


    return app
