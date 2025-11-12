# app/models/mixins.py
import datetime
from app import db
from app.models._query import SoftDeleteQuery  # <-- Import the new class

class SoftDeleteMixin:
    """
    Mixin for models to enable soft delete functionality
    using a 'deleted_at' timestamp.
    """
    deleted_at = db.Column(db.DateTime, nullable=True, default=None)

    # ... (Your delete, restore, and force_delete methods are unchanged) ...

    def delete(self, commit=True):
        self.deleted_at = datetime.datetime.utcnow()
        db.session.add(self)
        if commit:
            db.session.commit()

    def restore(self, commit=True):
        self.deleted_at = None
        db.session.add(self)
        if commit:
            db.session.commit()

    def force_delete(self, commit=True):
        db.session.delete(self)
        if commit:
            db.session.commit()

    @property
    def is_deleted(self):
        return self.deleted_at is not None

    # --- THIS IS THE NEW, CORRECT IMPLEMENTATION ---
    @classmethod
    @property
    def query(cls):
        """
        Custom .query property that automatically filters out
        soft-deleted items.
        
        This overrides the default db.Model.query property.
        """
        # 1. Create an instance of our custom query class
        query_obj = SoftDeleteQuery(cls, session=db.session)
        
        # 2. Apply the default filter
        return query_obj.filter(cls.deleted_at == None)