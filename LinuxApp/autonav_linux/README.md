# autonav_linux

A new Flutter project.

## Getting Started

This project is a starting point for a Flutter application.

A few resources to get you started if this is your first Flutter project:

- [Lab: Write your first Flutter app](https://docs.flutter.dev/get-started/codelab)
- [Cookbook: Useful Flutter samples](https://docs.flutter.dev/cookbook)

For help getting started with Flutter development, view the
[online documentation](https://docs.flutter.dev/), which offers tutorials,
samples, guidance on mobile development, and a full API reference.

# AutoNav Ubuntu Setup Guide

This guide covers:

* Creating and installing the AutoNav Plymouth boot theme
* Adding the AutoNav application as a system command
* Creating a desktop application shortcut
* Starting AutoNav automatically after login

---

# 1. Install Required Packages

Install Plymouth and required tools:

```bash
sudo apt update
sudo apt install -y plymouth plymouth-themes
```

---

# 2. Create the AutoNav Plymouth Theme

Create the theme directory:

```bash
sudo mkdir -p /usr/share/plymouth/themes/autonav
```

Copy the AutoNav logo into the theme directory:

```bash
sudo cp ~/autonav_icon.png /usr/share/plymouth/themes/autonav/logo.png
```

Create the Plymouth theme configuration:

```bash
sudo nano /usr/share/plymouth/themes/autonav/autonav.plymouth
```

Add:

```ini
[Plymouth Theme]
Name=AutoNav
Description=AutoNav Boot Splash Theme
ModuleName=script

[script]
ImageDir=/usr/share/plymouth/themes/autonav
ScriptFile=/usr/share/plymouth/themes/autonav/autonav.script
```

Save the file.

---

# 3. Create the Plymouth Logo Script

Create the script:

```bash
sudo nano /usr/share/plymouth/themes/autonav/autonav.script
```

Add:

```javascript
logo = Image("logo.png");

screen_width = Window.GetWidth();
screen_height = Window.GetHeight();

# Set logo width to 30% of the screen width
target_width = screen_width * 0.30;

# Maintain the original aspect ratio
target_height = logo.GetHeight() * target_width / logo.GetWidth();

# Scale the logo
logo = logo.Scale(target_width, target_height);

# Get the dimensions after scaling
logo_width = logo.GetWidth();
logo_height = logo.GetHeight();

# Center the logo
logo_x = (screen_width - logo_width) / 2;
logo_y = (screen_height - logo_height) / 2;

# Display the logo
logo_sprite = Sprite(logo);
logo_sprite.SetPosition(logo_x, logo_y, 10000);
```

```
logo = Image("logo.png");

logo_sprite = Sprite(logo);

fun update_logo()
{
    screen_width = Window.GetWidth();
    screen_height = Window.GetHeight();

    target_width = screen_width * 0.30;

    target_height = logo.GetHeight() * target_width / logo.GetWidth();

    scaled_logo = logo.Scale(target_width, target_height);

    logo_sprite.SetImage(scaled_logo);

    logo_width = scaled_logo.GetWidth();
    logo_height = scaled_logo.GetHeight();

    logo_x = (screen_width - logo_width) / 2;
    logo_y = (screen_height - logo_height) / 2;

    logo_sprite.SetPosition(logo_x, logo_y, 10000);
}

update_logo();

Plymouth.SetRefreshFunction(update_logo);
```

Save the file.

---

# 4. Enable the AutoNav Plymouth Theme

Set AutoNav as the active Plymouth theme:

```bash
sudo plymouth-set-default-theme autonav
```

Update the initramfs:

```bash
sudo update-initramfs -u
```

Reboot:

```bash
sudo reboot
```

---

# 5. Build the AutoNav Release Application

Go to the Flutter application directory:

```bash
cd ~/AutoNav/LinuxApp/autonav_linux
```

Build the Linux release application:

```bash
flutter build linux --release
```

The release bundle is located at:

```text
~/AutoNav/LinuxApp/autonav_linux/build/linux/x64/release/bundle
```

---

# 6. Create a System Command for AutoNav

Create a symbolic link to the application executable:

```bash
sudo ln -sf \
~/AutoNav/LinuxApp/autonav_linux/build/linux/x64/release/bundle/autonav_linux \
/usr/local/bin/autonav
```

Make the executable executable:

```bash
chmod +x \
~/AutoNav/LinuxApp/autonav_linux/build/linux/x64/release/bundle/autonav_linux
```

The application can now be started using:

```bash
autonav
```

The symbolic link points directly to the Flutter release build.

When a new release is built at the same location, the `autonav` command automatically uses the updated executable.

---

# 7. Add AutoNav as a Desktop Application

Copy the application icon to the system icon location:

```bash
sudo cp ~/autonav_icon.png /usr/share/pixmaps/autonav.png
```

Create the desktop application file:

```bash
sudo nano /usr/share/applications/autonav.desktop
```

Add:

```ini
[Desktop Entry]
Version=1.0
Type=Application
Name=AutoNav
Comment=AutoNav Robot Application
Exec=/usr/local/bin/autonav
Icon=/usr/share/pixmaps/autonav.png
Terminal=false
Categories=Utility;
StartupNotify=true
```

Save the file.

Update desktop application information:

```bash
sudo update-desktop-database /usr/share/applications
```

AutoNav will now appear in the Ubuntu Applications menu.

---

# 8. Configure Automatic Login

Edit the GDM configuration:

```bash
sudo nano /etc/gdm3/custom.conf
```

Under the `[daemon]` section, add:

```ini
[daemon]
AutomaticLoginEnable=true
AutomaticLogin=autonav
```

Save the file.

---

# 9. Keep GNOME Wayland Enabled

Do not add the following configuration:

```ini
WaylandEnable=false
```

The system should continue using the default Ubuntu GNOME Wayland session.

---

# 10. Remove Openbox Configuration

Remove Openbox if it was previously installed for AutoNav:

```bash
sudo apt remove --purge -y openbox libobrender32 libobt2
```

Remove unused packages:

```bash
sudo apt autoremove --purge -y
```

Remove old Openbox startup configuration:

```bash
rm -rf ~/.config/openbox
```

Remove the old AutoNav restart script:

```bash
rm -f ~/start-autonav.sh
```

If Openbox was configured as the user session, edit:

```bash
sudo nano /var/lib/AccountsService/users/autonav
```

Remove:

```ini
Session=openbox
```

---

# 11. Configure AutoNav Autostart

Create the GNOME autostart directory:

```bash
mkdir -p ~/.config/autostart
```

Create the AutoNav autostart file:

```bash
nano ~/.config/autostart/autonav.desktop
```

Add:

```ini
[Desktop Entry]
Type=Application
Name=AutoNav
Comment=Start AutoNav automatically
Exec=/usr/local/bin/autonav
Terminal=false
X-GNOME-Autostart-enabled=true
StartupNotify=false
```

Save the file.

AutoNav will now start automatically when the `autonav` user logs into GNOME.

When the AutoNav application closes, GNOME remains running and the normal Ubuntu desktop becomes available.

---

# 12. Configure a Passwordless Keyring

Automatic login does not provide the user password to GNOME Keyring.

Install the keyring manager:

```bash
sudo apt install -y seahorse
```

Open Seahorse:

```bash
seahorse
```

In Seahorse:

1. Open **Passwords**
2. Select the **Login** keyring
3. Right-click the keyring
4. Select **Change Password**
5. Enter the current keyring password
6. Leave the new password empty
7. Leave the confirmation password empty
8. Accept the warning about storing secrets without encryption

This prevents the keyring password prompt during automatic login.

---

# 13. Reboot

Restart the system:

```bash
sudo reboot
```
