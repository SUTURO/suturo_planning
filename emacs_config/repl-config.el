
(custom-set-faces
  '(font-lock-type-face ((t (:bold nil :foreground "pink")))) ;;(rosy brown)
  '(font-lock-negation-char-face ((t (:bold nil :foreground "pale violet red")))))


(defun my-lisp-mode-hook ()
  (font-lock-add-keywords nil
    '(("\\_<\\(\\?[-_A-Za-z0-9]+*\\)\\_>" (0 font-lock-warning-face))
      ("\\_<\\([<][-]*\\)\\_>" (0 font-lock-preprocessor-face))
      ("\\_<\\(prolog:[<][-]*\\)\\_>" (0 font-lock-preprocessor-face))
      ("\\_<\\(prolog:[-_A-Za-z0-9]+*\\)\\_>" (0 font-lock-preprocessor-face))

      ("\\_<\\(cram-prolog:[-][>]\\)\\_>" (0 font-lock-preprocessor-face))
      ("\\_<\\(cram-prolog:[<][-]\\)\\_>" (0 font-lock-preprocessor-face))
      ("\\_<\\(cram-prolog:[-_A-Za-z0-9]+*\\)\\_>" (0 font-lock-preprocessor-face))

      ("\\_<\\(spec:[-_A-Za-z0-9]+*\\)\\_>" (0 font-lock-type-face)) ;; ido-virtual-face
      ("\\_<\\(lisp-fun\\)\\_>" (0 font-lock-negation-char-face))
      ("\\_<\\(desig:[-_A-Za-z0-9]+*\\)\\_>" (0 font-lock-type-face))
      ("\\_<\\(kvr::[-_A-Za-z0-9]+*\\)\\_>" (0 font-lock-negation-char-face))      
      )))
(add-hook 'slime-mode-hook #'my-lisp-mode-hook)

;;; General configuration
(customize-set-variable 'indent-tabs-mode nil)
(setq default-tab-width 2)
(global-font-lock-mode t)
(setq query-replace-highlight t)
(setq search-highlight t)
(show-paren-mode 1)
(global-set-key '[delete] 'delete-char)
(setq minibuffer-max-depth nil)
(autoload 'mwheel-install "mwheel" "Enable mouse wheel support.")
;; Enable copy-pasting between programs (Kill-ring <-> x11)
(setq x-select-enable-clipboard t)
(cond
 ((fboundp 'x-cut-buffer-or-selection-value)
  (setq interprogram-paste-function 'x-cut-buffer-or-selection-value))
 ((fboundp 'x-selection-value) ;; emacs 24 or later
  (setq interprogram-paste-function 'x-selection-value))
 (t (setq x-select-enable-clipboard nil))) ;; no connection to X server

;;; Start slime
;; ``slime-config`` is located in the ``slime_ros`` package.
;; It's path is passed to emacs through the -L argument of
;; the ``roslisp_repl`` executable.
(require 'slime-config)

;; The following gets rid of the gray highlighting of uncompiled code
;; that can be confusing and annoying for the beginner Lispers.
(add-hook 'slime-mode-hook (lambda () (slime-highlight-edits-mode 0)))

;; some key bindings to, e.g., enable easy autocompletion etc.
(eval-after-load 'slime
  '(progn
     ;; Fix for M-, when using it with dired and A
     (define-key slime-mode-map (kbd "M-,")
       (lambda ()
         (interactive)
         (condition-case nil
             (slime-pop-find-definition-stack)
           (error (tags-loop-continue)))))
     (global-set-key "\C-cs" 'slime-selector)
     (define-key slime-repl-mode-map (kbd "C-M-<backspace>")
       'slime-repl-delete-current-input)
     (define-key slime-mode-map "\r" 'newline-and-indent)
     (define-key slime-mode-map [tab]
       (lambda ()
         (interactive)
         (let ((yas-fallback-behavior nil))
           (unless (yas-expand)
             (slime-fuzzy-indent-and-complete-symbol)))))
     (define-key slime-mode-map (kbd "M-a")
       (lambda ()
         (interactive)
         (let ((ppss (syntax-ppss)))
           (if (nth 3 ppss)
               (goto-char (1+ (nth 8 ppss)))
             (progn
               (backward-up-list 1)
               (down-list 1))))))
     (define-key slime-mode-map (kbd "M-e")
       (lambda ()
         (interactive)
         (let ((ppss (syntax-ppss)))
           (if (nth 3 ppss)
               (progn
                 (goto-char (nth 8 ppss))
                 (forward-sexp 1)
                 (backward-char 1))
             (progn
               (up-list 1)
               (backward-down-list 1))))))))

;;; [ and ] should be handled paranthesis-like in lisp files.
(modify-syntax-entry ?\[ "(]  " lisp-mode-syntax-table)
(modify-syntax-entry ?\] ")[  " lisp-mode-syntax-table)

(slime)

(delete-other-windows)

;;; Footer
(provide 'repl-config)


;;; OWN ADJUSTMENTS
;;; theme: 
(load-theme 'wombat)

;;; line numbers
(global-linum-mode t)

;;; 80 char ruler
(require 'fill-column-indicator)

;;;; settings
(setq-default fci-rule-column 80)
(setq fci-rule-use-dashes t)
(setq fci-dash-pattern 0.5)
(setq fci-rule-color "darkred")

;(fci-rule-color "darkgray")

;;;; enable by default
(define-globalized-minor-mode global-fci-mode fci-mode (lambda () (fci-mode 1)))

(global-fci-mode 1)

;;; highlighting todo
(require 'hl-todo)

(global-hl-todo-mode 1)

;;;; shortcut commands
(define-key hl-todo-mode-map (kbd "C-c p") 'hl-todo-previous)
(define-key hl-todo-mode-map (kbd "C-c n") 'hl-todo-next)
(define-key hl-todo-mode-map (kbd "C-c o") 'hl-todo-occur)

;;; Highlight and overwrite selected regions (CUA mode) (from Gayas tutorial)
(cua-mode 1)
(setq cua-enable-cua-keys nil)
(setq cua-enable-modeline-indications t)
(setq cua-remap-control-v nil)
(setq cua-remap-control-z nil)

;;; Don't clutter the directories with *~ backup files
;;; and automatically overwrite them up to a number of backup versions
(setq backup-directory-alist (quote ((".*" . "~/.emacs.d/emacs-file-backups"))))
(setq delete-old-versions t)
(setq version-control t)
(setq kept-new-versions 3)
(setq kept-old-versions 3)

;;; Enable undo on window configuration: C-c left (undo) and C-c right (redo)
(winner-mode)

;;; Enable tooltips: bubbles with help text
(gud-tooltip-mode t)

;;; Enable autocompletion suggestions for minibuffer
(icomplete-mode 1)

;;; When opening Lisp files, don't ask if the variables are safe for this list.
;;; These are the variables from slime.
(setq safe-local-variable-values
      (quote ((TeX-PDF . t) (readtable . nisp) (readtable . :nisp)
              (Package . NISP) (Syntax . Common-Lisp) (Package . SAX)
              (Encoding . utf-8) (Syntax . COMMON-LISP) (Package . CL-PPCRE)
              (package . rune-dom) (readtable . runes)
              (Syntax . ANSI-Common-Lisp) (Base . 10) (lexical-binding . t))))

;;; Autocomplete in the minibuffer for filenames etc.
(require 'ido)
(ido-mode 'both)
(ido-everywhere 1)
(setq ido-completion-buffer-all-completions t)
(setq ido-auto-merge-delay-time 2)
(setq ido-default-buffer-method (quote selected-window))
(setq ido-default-file-method (quote samewindow))
(setq ido-enable-dot-prefix t)
(setq ido-enable-flex-matching t)
(setq ido-max-window-height 5)
(setq ido-read-file-name-as-directory-commands (quote (find-dired)))
(setq ido-show-dot-for-dired t)
(setq ido-use-filename-at-point (quote guess))
(setq ido-use-url-at-point t)


;;; Adjust the indentation for MAKE-INSTANCE
(put 'make-instance 'common-lisp-indent-function '(4 &rest 2))



