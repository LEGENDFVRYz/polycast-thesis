from app import db


class Session(db.Model):
    """
    Represents an session of notes per gallery
    """
    __tablename__ = "sessions"
    id = db.Column('id', db.Integer, primary_key=True)
    
    # Foreign Key to Gallery (Many Sessions to One Gallery)
    gallery_id = db.Column(db.Integer, db.ForeignKey('galleries.id', ondelete='CASCADE'), nullable=False)
    
    name = db.Column(db.String(80), nullable=False)
    created_at = db.Column(db.DateTime, default=db.func.now(), nullable=False)

    # Constraints
    __table_args__ = (
        db.UniqueConstraint('gallery_id', 'name', name='uq_gallery_session_name'),
    )