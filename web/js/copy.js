/* Copy buttons on command blocks. */
(() => {
  "use strict";
  document.querySelectorAll(".copy").forEach((btn) => {
    btn.addEventListener("click", async () => {
      const code = btn.parentElement.querySelector("code");
      try {
        await navigator.clipboard.writeText(code.textContent);
        btn.textContent = "Copied";
      } catch {
        const r = document.createRange();
        r.selectNodeContents(code);
        const s = getSelection(); s.removeAllRanges(); s.addRange(r);
        btn.textContent = "Press Ctrl+C";
      }
      setTimeout(() => (btn.textContent = "Copy"), 1800);
    });
  });
})();
