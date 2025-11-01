from app import db
from app.models.admin import Admin


def register_admin(username: str, password: str) -> bool:
    """Register a new admin (returns True if successful)."""
    existing = Admin.query.filter_by(username=username).first()
    if existing:
        return False

    user = Admin(username=username)
    user.set_password(password)
    db.session.add(user)
    db.session.commit()
    return True

def verify_admin(username: str, password: str):
    """Verify credentials and return user object if valid."""
    user = Admin.query.filter_by(username=username).first()
    if user and user.check_password(password):
        return user  # return the user object
    return None