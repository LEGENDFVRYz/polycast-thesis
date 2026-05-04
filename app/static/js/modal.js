document.addEventListener('DOMContentLoaded', function() {

    // ===========================================
    // 1. GENERIC MODAL OPEN/CLOSE LOGIC
    // ===========================================

    // --- Open Modals (delegated — handles dynamically-injected buttons) ---
    document.addEventListener('click', e => {
        const button = e.target.closest('[data-modal-target]');
        if (!button) return;
        const modal = document.querySelector(button.dataset.modalTarget);
        if (modal) openModal(modal);
    });

    // --- Close Modals (with '[data-close-modal]' attribute) ---
    // Find all buttons (like the 'x') with [data-close-modal]
    document.querySelectorAll('[data-close-modal]').forEach(button => {
        button.addEventListener('click', () => {
            const modal = button.closest('.modal-overlay');
            if (modal) {
                closeModal(modal);
            }
        });
    });

    // --- Close Modals (by clicking background) ---
    document.querySelectorAll('.modal-overlay').forEach(modal => {
        modal.addEventListener('click', event => {
            // Check if the click was on the overlay itself, not the content
            // Don't close the gallery deleted modal by clicking outside
            if (event.target === modal && modal.id !== 'galleryDeletedModal') {
                closeModal(modal);
            }
        });
    });

    // --- Helper Functions ---
    function openModal(modal) {
        if (modal == null) return;
        modal.classList.remove('closing');
        modal.style.display = 'flex';
    }

    function closeModal(modal) {
        if (modal == null) return;
        
        // Add closing class for smooth exit animation
        modal.classList.add('closing');
        
        // Wait for animation to finish before hiding
        setTimeout(() => {
            modal.style.display = 'none';
            modal.classList.remove('closing');
        }, 250); // Match the CSS animation duration
    }
});