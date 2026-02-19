let lastscroll = window.pageYOffset;
const header = document.querySelector("header.app-wrapper");
let ticking = false;
const delta = 5;    // minimum scroll difference

// Animation In
window.addEventListener("DOMContentLoaded", () => {
    header.classList.add("animate-in");
});

// Tracking 
window.addEventListener("scroll", () => {
    if (!ticking) {
        window.requestAnimationFrame(() => {
            let current = window.pageYOffset;
            const diff = Math.abs(current - lastscroll);
            
            if (diff > delta) {
                if (current > lastscroll && current > 50) {
                    header.classList.add("hide");
                } else {
                    header.classList.remove("hide");
                }
                
                lastscroll = current;
            }

            // resets
            ticking = false;
        });

        // skips
        ticking = true;
    }
});