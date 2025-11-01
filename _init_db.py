from app import create_app, db

# Create an app instance
app = create_app()

with app.app_context():
    # Now we can run create_all()
    db.create_all()
    print("Database tables created successfully!")