from app import db
import bcrypt

class Admin(db.Model):
    """
    Represents an authenticated user/admin account.
    """
    __tablename__ = "admins"

    _id = db.Column('id', db.Integer, primary_key=True)
    username = db.Column(db.String(80), unique=True, nullable=False)
    password_hash = db.Column(db.LargeBinary(128), nullable=False)  # bcrypt hash stored as bytes

    def set_password(self, password: str):
        """Hash and store the user's password."""
        self.password_hash = bcrypt.hashpw(password.encode('utf-8'), bcrypt.gensalt())

    def check_password(self, password: str) -> bool:
        """Verify a password against the stored hash."""
        return bcrypt.checkpw(password.encode('utf-8'), self.password_hash)
