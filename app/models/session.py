from app import db


class Session(db.Model):
    """
    Represents an session of notes per gallery
    """
    __tablename__ = "sessions"
    id = db.Column('id', db.Integer, primary_key=True)
    
    # Foreign Key to Gallery (Many Sessions to One Gallery)
    gallery_id = db.Column(db.Integer, db.ForeignKey('galleries.id'), nullable=False)
    
    name = db.Column(db.String(80), unique=True, nullable=False)
    created_at = db.Column(db.DateTime, default=db.func.now(), nullable=False)