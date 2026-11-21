/* Light/dark toggle. The saved choice is applied in <head> before first paint. */
(() => {
  "use strict";
  const toggle = document.getElementById("theme-toggle");
  const root = document.documentElement;
  const meta = document.querySelector('meta[name="theme-color"]');
  const sync = () => {
    const dark = root.dataset.theme === "dark";
    toggle.setAttribute("aria-pressed", String(dark));
    meta.content = dark ? "#0A121A" : "#F2F5F7";
  };
  sync();
  toggle.addEventListener("click", () => {
    const dark = root.dataset.theme !== "dark";
    if (dark) root.dataset.theme = "dark"; else delete root.dataset.theme;
    try { localStorage.setItem("theme", dark ? "dark" : "light"); } catch (e) {}
    sync();
    document.dispatchEvent(new CustomEvent("themechange"));
  });
})();
