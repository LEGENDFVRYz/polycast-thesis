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

def verify_admin(username: str, password: str) -> bool:
    """Verify credentials."""
    user = Admin.query.filter_by(username=username).first()
    if not user:
        return False
    return user.check_password(password)
