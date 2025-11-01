from app import db


class Gallery(db.Model):
    """
    Represents a gallery that hosts multiple sessions
    """
    __tablename__ = "galleries"
    
    id = db.Column('id', db.Integer, primary_key=True)
    
    # Foreign Key to User (Many Gallery to One Admin)
    admin_id = db.Column(db.Integer, db.ForeignKey('admins.id'), nullable=False)

    name = db.Column(db.String(80), unique=True, nullable=False)
    
    # Relationships
    sessions = db.relationship('Session', backref='gallery', lazy=True, cascade="all, delete-orphan")