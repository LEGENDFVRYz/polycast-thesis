from app import db


class Gallery(db.Model):
    """
    Represents a gallery that hosts multiple sessions
    """
    __tablename__ = "galleries"
    
    id = db.Column('id', db.Integer, primary_key=True)
    # Foreign Key to User (Many Gallery to One Admin)
    admin_id = db.Column(db.Integer, db.ForeignKey('admins.id'), nullable=False)
    name = db.Column(db.String(80), nullable=False)
    
    # Relationships
    sessions = db.relationship(
        'Session',
        backref='gallery',
        lazy=True,
        cascade="all, delete-orphan",
        passive_deletes=True
    )
    
    # Constraint
    __table_args__ = (
        db.UniqueConstraint('admin_id', 'name', name='uq_admin_gallery_name'),
    )
    
    
    def to_dict(self):
        return {
            "id": self.id,
            "name": self.name,
            "admin_id": self.admin_id
        }
    
    def get_gallery_name(self):
        return self.name