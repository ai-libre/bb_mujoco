defmodule BbMujocoWeb.PageController do
  use BbMujocoWeb, :controller

  def home(conn, _params) do
    render(conn, :home)
  end
end
